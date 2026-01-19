#!/usr/bin/env python3
"""
Command-line interface for pickn_place sequences with JSON-driven recipes.
"""
import argparse
import json
import sys
from pathlib import Path

# Import sequence functions
try:
    from oms_v1.sequences.home      import SEQUENCES as HOME_SEQ
    from oms_v1.sequences.cups      import SEQUENCES as CUPS_SEQ
    from oms_v1.sequences.espresso import SEQUENCES as ESPR_SEQ
    from oms_v1.sequences.cleaning  import SEQUENCES as CLEAN_SEQ
    from oms_v1.sequences.test      import SEQUENCES as TEST_SEQ
except ImportError as e:
    print(f"[ERROR] Could not import sequences: {e}")
    sys.exit(1)

# Merge all sequence mappings
SEQUENCES = {}
SEQUENCES.update(HOME_SEQ)
SEQUENCES.update(CUPS_SEQ)
SEQUENCES.update(ESPR_SEQ)
SEQUENCES.update(CLEAN_SEQ)
SEQUENCES.update(TEST_SEQ)

# Map action names to actual callables
ACTION_MAP = {}
for name, fn in SEQUENCES.items():
    ACTION_MAP[name] = fn


def run_recipe(recipe_name: str, recipe_file: Path):
    """
    Load and execute a JSON-defined recipe by name.
    Each step in JSON must have an 'action' matching a key in ACTION_MAP,
    and any additional fields become keyword arguments to the function.
    """
    if not recipe_file.exists():
        print(f"[ERROR] Recipe file not found: {recipe_file}")
        sys.exit(1)
    data = json.loads(recipe_file.read_text())
    steps = data.get(recipe_name)
    if not steps:
        print(f"[ERROR] Recipe '{recipe_name}' not defined in {recipe_file}")
        sys.exit(1)
    for step in steps:
        action = step.get('action')
        fn = ACTION_MAP.get(action)
        if fn is None:
            print(f"[ERROR] No sequence function mapped for action '{action}'")
            sys.exit(1)
        # Prepare kwargs (remove 'action')
        kwargs = {k: v for k, v in step.items() if k != 'action'}
        print(f"▶ Executing {action}({', '.join(f'{k}={v}' for k,v in kwargs.items())})")
        try:
            fn(**kwargs)
        except Exception as e:
            print(f"[ERROR] '{action}' raised exception: {e}")
            sys.exit(1)
    print(f"✅ Recipe '{recipe_name}' completed.")


def interactive_menu():
    """
    Fallback interactive menu to manually select sequences.
    """
    names = list(SEQUENCES.keys())
    while True:
        print("\n🔧 Available Sequences:")
        for idx, name in enumerate(names, start=1):
            print(f"  {idx}) {name}")
        print("  q) Quit\n")

        choice = input("Which? ").strip()
        if choice.lower() in ('q', 'quit', 'exit'):
            print("Goodbye!")
            sys.exit(0)
        if choice.isdigit():
            idx = int(choice) - 1
            if 0 <= idx < len(names):
                seq = names[idx]
            else:
                print(f"[ERROR] Invalid selection: {choice}")
                continue
        else:
            seq = choice
        fn = SEQUENCES.get(seq)
        if not fn:
            print(f"[ERROR] Unknown sequence: {seq}")
            continue
        # Prompt for parameters
        sig = __import__('inspect').signature(fn)
        kwargs = {}
        for param in sig.parameters.values():
            val = input(f"{param.name}? ").strip()
            kwargs[param.name] = val
        print(f"\n▶ Running '{seq}' with arguments: {kwargs}\n")
        try:
            fn(**kwargs)
        except Exception as e:
            print(f"[ERROR] Sequence '{seq}' raised an exception: {e}")


def main():
    parser = argparse.ArgumentParser(
        description='CLI for oms_v1 sequences, supports JSON recipes.'
    )
    parser.add_argument('--recipe', '-r',
                        help='Name of the recipe to run from JSON file')
    parser.add_argument('--recipes-file', '-f',
                        default=str(Path(__file__).parent.parent / 'recipes.json'),
                        help='Path to recipes JSON')
    args = parser.parse_args()

    if args.recipe:
        run_recipe(args.recipe, Path(args.recipes_file))
    else:
        interactive_menu()


if __name__ == '__main__':
    main()