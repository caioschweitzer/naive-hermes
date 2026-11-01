import sys
from pathlib import Path

# Adiciona o diretório proto ao sys.path para imports gerados pelo protoc
proto_dir = str(Path(__file__).parent)
if proto_dir not in sys.path:
    sys.path.append(proto_dir)
