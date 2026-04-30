#!/usr/bin/env python3
import sys
import time


def main():
    prefix = sys.argv[1] if len(sys.argv) > 1 else "rerun"
    sys.stdout.write(f"{prefix}_{time.strftime('%Y%m%d_%H%M%S')}")


if __name__ == "__main__":
    main()
