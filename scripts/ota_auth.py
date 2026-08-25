Import("env")

import ast
import re
from pathlib import Path


project_dir = Path(env.subst("$PROJECT_DIR"))
secrets_path = project_dir / "include" / "secrets.h"

if not secrets_path.is_file():
    raise RuntimeError(
        "include/secrets.h est manquant: impossible de configurer l'authentification OTA"
    )

secrets_text = secrets_path.read_text(encoding="utf-8")
match = re.search(
    r'^\s*#define\s+SECRET_OTA_PASSWORD\s+("(?:\\.|[^"\\])*")',
    secrets_text,
    re.MULTILINE,
)
if not match:
    raise RuntimeError("SECRET_OTA_PASSWORD est absent de include/secrets.h")

try:
    ota_password = ast.literal_eval(match.group(1))
except (SyntaxError, ValueError) as exc:
    raise RuntimeError("SECRET_OTA_PASSWORD n'est pas une chaine C valide") from exc

if not ota_password:
    raise RuntimeError("SECRET_OTA_PASSWORD ne doit pas etre vide")
if not re.fullmatch(r"[A-Za-z0-9@+_,.~-]+", ota_password):
    raise RuntimeError(
        "SECRET_OTA_PASSWORD contient un caractere non sur pour la ligne de commande espota"
    )

# Le meme secret configure ArduinoOTA dans le firmware et espota lors de l'upload.
# Il reste dans le fichier gitignore include/secrets.h et n'est jamais duplique ici.
env.Append(UPLOADERFLAGS=[f"--auth={ota_password}", "--port=3232"])
