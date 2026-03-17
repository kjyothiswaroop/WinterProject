#!/usr/bin/env python3
"""Quick test to see how CLIP matches spoken queries to object names."""

import numpy as np
import torch
from transformers import CLIPModel, CLIPTokenizer

OBJECTS = ["white fridge", "yellow board", "kitchen"]

QUERIES = [
    "go to the white fridge",
    "take me to the fridge",
    "I need a cold drink",
    "where is the refrigerator",
    "go to the kitchen",
    "I want to cook something",
    "take me to the yellow board",
    "I want to write something on the board",
    "where can I eat",
    "navigate to the whiteboard",
    "hello world",
]

THRESHOLD = 0.70


def main():
    print("Loading CLIP model...")
    model_name = "openai/clip-vit-base-patch32"
    model = CLIPModel.from_pretrained(model_name)
    tokenizer = CLIPTokenizer.from_pretrained(model_name)
    model.eval()

    with torch.no_grad():
        def embed(texts):
            inputs = tokenizer(texts, padding=True, return_tensors="pt")
            out = model.text_model(**inputs)
            emb = model.text_projection(out.pooler_output)
            emb = emb / emb.norm(dim=-1, keepdim=True)
            return emb.numpy()

        # Embed object names
        obj_emb = embed(OBJECTS)

        print(f"\nObjects: {OBJECTS}")
        print(f"Threshold: {THRESHOLD}")
        print("-" * 80)

        for query in QUERIES:
            q_emb = embed([query])

            sims = (q_emb @ obj_emb.T).squeeze(0)
            best_idx = int(np.argmax(sims))
            best_score = float(sims[best_idx])
            matched = best_score >= THRESHOLD

            scores_str = "  ".join(
                f"{name}: {sims[i]:.3f}" for i, name in enumerate(OBJECTS)
            )

            status = "MATCH" if matched else "REJECT"
            print(f'\n  Query: "{query}"')
            print(f"  Scores: {scores_str}")
            print(f"  Result: [{status}] {OBJECTS[best_idx]} ({best_score:.3f})")


if __name__ == "__main__":
    main()
