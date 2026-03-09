import socket
import struct
import json
import tempfile
import wave

from faster_whisper import WhisperModel
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
from unitree_api.msg import Request


MULTICAST_GROUP = "239.168.123.161"
MULTICAST_PORT = 5555
PACKET_SIZE = 1024
PACKET_DURATION = 0.032  # seconds per packet (1024 bytes / 2 bytes per sample / 16000 Hz)
API_VOICE_SET_MODE = 1008


class VoiceQuery(Node):

    def __init__(self):
        super().__init__("voice_query")

        self.declare_parameter("recording_duration", 8.0)
        self.record_duration = self.get_parameter("recording_duration").get_parameter_value().double_value

        self.voice_req_pub = self.create_publisher(Request, "/api/voice/request", 10)

        self.query_pub = self.create_publisher(String, "/semantic_nav/query", 10)
        self.voice_srv = self.create_service(Trigger, "voice_query", self.handle_query)
        self.create_subscription(String, "/semantic_nav/tts", self._on_tts, 10)

        self.model = WhisperModel("base", device="cpu", compute_type="int8")

        # Allow publisher to connect before sending
        import time; time.sleep(0.5)
        self._set_mic_mode(1)

        self.get_logger().info("Voice Query Node started! Ready to Listen.......")

    def _set_mic_mode(self, mode: int):
        req = Request()
        req.header.identity.api_id = API_VOICE_SET_MODE
        req.parameter = json.dumps({"mode": mode})
        self.voice_req_pub.publish(req)
        self.get_logger().info(f"Mic mode set to {mode}")

    def record(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", MULTICAST_PORT))
        mreq = struct.pack("4s4s", socket.inet_aton(MULTICAST_GROUP), socket.inet_aton("192.168.123.220"))
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        num_packets = int(self.record_duration / PACKET_DURATION)
        chunks = []

        self.get_logger().info(f"Recording for {self.record_duration}s ({num_packets} packets)...")
        for _ in range(num_packets):
            data, _ = sock.recvfrom(PACKET_SIZE)
            chunks.append(data)

        sock.close()
        return b"".join(chunks)

    def transcribe(self, pcm_bytes):
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            with wave.open(tmp.name, "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(16000)
                wf.writeframes(pcm_bytes)

        self.get_logger().info("Transcribing...")
        segments, _ = self.model.transcribe(tmp.name)
        return " ".join(s.text for s in segments).strip()

    def _say(self, text: str):
        req = Request()
        req.header.identity.api_id = 1001
        req.parameter = json.dumps({"index": 0, "text": text, "speaker_id": 1})
        self.voice_req_pub.publish(req)

    def _on_tts(self, msg: String):
        self._say(msg.data)

    def handle_query(self, request, response):
        pcm_bytes = self.record()
        text = self.transcribe(pcm_bytes)

        msg = String()
        msg.data = text.lower()
        self.query_pub.publish(msg)

        self.get_logger().info(f"Transcribed: '{text}'")
        response.success = True
        response.message = text
        return response

    def destroy_node(self):
        self._set_mic_mode(2)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceQuery()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
