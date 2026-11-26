import argparse
import subprocess
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="Save the live /map as image and YAML")
    parser.add_argument("--output", required=True, help="Map basename, without an extension")
    args = parser.parse_args()
    output = Path(args.output).expanduser().resolve()
    if any(Path(str(output) + suffix).exists() for suffix in (".yaml", ".pgm", ".png")):
        parser.error("Map already exists; choose a new output basename")
    output.parent.mkdir(parents=True, exist_ok=True)
    result = subprocess.run([
        "ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(output), "-t", "/map",
    ], check=False)
    if result.returncode:
        return result.returncode
    if (not Path(str(output) + ".yaml").is_file() or
            not any(Path(str(output) + suffix).is_file() for suffix in (".pgm", ".png"))):
        parser.exit(1, "Map saver returned without both metadata and image; inspect its output\n")
    print("Saved map metadata: " + str(output) + ".yaml")
    return 0
