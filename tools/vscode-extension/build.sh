#!/usr/bin/env bash
# Packages this folder into a .vsix without needing npm or vsce.
set -euo pipefail
cd "$(dirname "$0")"

name=$(sed -n 's/^  "name": *"\([^"]*\)".*/\1/p' package.json)
publisher=$(sed -n 's/^  "publisher": *"\([^"]*\)".*/\1/p' package.json)
version=$(sed -n 's/^  "version": *"\([^"]*\)".*/\1/p' package.json)
engine=$(sed -n 's/^ *"vscode": *"\([^"]*\)".*/\1/p' package.json)
out="$PWD/$publisher-$name.vsix"

staging=$(mktemp -d)
trap 'rm -rf "$staging"' EXIT
mkdir "$staging/extension"
cp -R package.json extension.js icons "$staging/extension/"

cat > "$staging/[Content_Types].xml" <<'EOF'
<?xml version="1.0" encoding="utf-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension=".json" ContentType="application/json"/>
  <Default Extension=".js" ContentType="application/javascript"/>
  <Default Extension=".svg" ContentType="image/svg+xml"/>
  <Default Extension=".vsixmanifest" ContentType="text/xml"/>
</Types>
EOF

cat > "$staging/extension.vsixmanifest" <<EOF
<?xml version="1.0" encoding="utf-8"?>
<PackageManifest Version="2.0.0" xmlns="http://schemas.microsoft.com/developer/vsx-schema/2011">
  <Metadata>
    <Identity Language="en-US" Id="$name" Version="$version" Publisher="$publisher"/>
    <DisplayName>Churrobots Robot Buttons</DisplayName>
    <Description xml:space="preserve">Status bar buttons to start Claude, the robot simulator, and the dashboard.</Description>
    <Categories>Other</Categories>
    <Properties>
      <Property Id="Microsoft.VisualStudio.Code.Engine" Value="$engine"/>
    </Properties>
  </Metadata>
  <Installation>
    <InstallationTarget Id="Microsoft.VisualStudio.Code"/>
  </Installation>
  <Dependencies/>
  <Assets>
    <Asset Type="Microsoft.VisualStudio.Code.Manifest" Path="extension/package.json" Addressable="true"/>
  </Assets>
</PackageManifest>
EOF

rm -f "$out"
(cd "$staging" && zip -qrX "$out" "[Content_Types].xml" extension.vsixmanifest extension)
echo "Built $out"
