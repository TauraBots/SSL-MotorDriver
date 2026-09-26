"""Generate the firmware-only C representation of the files in web/."""

from pathlib import Path

Import("env")  # type: ignore[name-defined]  # Provided by PlatformIO/SCons.

project_dir = Path(env.subst("$PROJECT_DIR"))  # type: ignore[name-defined]
output = project_dir / ".pio" / "generated" / "web_assets.h"
assets = (
    ("web_index_html", project_dir / "web" / "index.html"),
    ("web_style_css", project_dir / "web" / "style.css"),
    ("web_app_js", project_dir / "web" / "app.js"),
)


def declaration(name: str, path: Path) -> str:
    data = path.read_bytes()
    values = [f"0x{byte:02x}" for byte in data]
    lines = [", ".join(values[index:index + 16]) for index in range(0, len(values), 16)]
    body = ",\n    ".join(lines)
    return (
        f"static const unsigned char {name}[] = {{\n    {body}\n}};\n"
        f"#define {name.upper()}_LEN ((size_t){len(data)}U)\n"
    )


generated = "#pragma once\n\n#include <stddef.h>\n\n"
generated += "\n".join(declaration(name, path) for name, path in assets)
output.parent.mkdir(parents=True, exist_ok=True)
if not output.exists() or output.read_text(encoding="utf-8") != generated:
    output.write_text(generated, encoding="utf-8", newline="\n")
