import shutil
from pathlib import Path
import click
import subprocess

pwd = Path(__file__).parent

files = ["hr_data"]

kaitai = shutil.which("kaitai-struct-compiler")
dot = shutil.which("dot")

if not kaitai:
    raise RuntimeError("kaitai-struct-compiler not found in PATH")

if not dot:
    raise RuntimeError("Graphviz (dot) not found in PATH")


# kaitai-struct-compiler -t graphviz -d $SCRIPT_DIR $SCRIPT_DIR/spot.ksy
# dot -Tpng $SCRIPT_DIR/spot.dot > $FIGURE_DIR/spot.png
def gen(file:str, src_dir:Path, out_dir:Path):
  input_file = src_dir / f"{file}.ksy"
  if not input_file.exists():
    raise RuntimeError(f"Input file {input_file} does not exist")
  subprocess.run([kaitai, "-t", "graphviz", "--outdir", str(out_dir.absolute()), str(input_file.absolute())], check=True)
  subprocess.run([dot, "-Tpng", str(out_dir / f"{file}.dot"), "-o", str(out_dir / f"{file}.png")], check=True)

@click.command()
@click.option("--src", "-s", type=click.Path(exists=True, file_okay=False), default=pwd)
@click.option("--dst", "-d", type=click.Path(exists=True, file_okay=False), default=pwd / "figures")
def main(src:str, dst:str):
  src_dir = Path(src)
  dst_dir = Path(dst)
  if not dst_dir.exists():
    dst_dir.mkdir(parents=True)
  for file in files:
    gen(file, src_dir, dst_dir)

main()
