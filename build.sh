#!/bin/sh
cd `dirname $0`

VENV_NAME="venv"
PYTHON="$VENV_NAME/bin/python"

if ! [ -x "$PYTHON" ]; then
    ./setup.sh || exit 1
fi

echo "Installing module dependencies before packaging..."
if ! $PYTHON -m pip install -r requirements.txt -Uqq; then
    exit 1
fi

if ! $PYTHON -m pip install pyinstaller -Uqq; then
    exit 1
fi

$PYTHON -m PyInstaller --onefile --clean \
    --paths src \
    --hidden-import="googleapiclient" \
    --collect-all viam \
    --copy-metadata viam-sdk \
    src/main.py

TAR_FILES="meta.json ./dist/main"
FIRST_RUN=$($PYTHON -c "import json; print(json.load(open('meta.json')).get('first_run', ''))" 2>/dev/null)
if [ -n "$FIRST_RUN" ] && [ -f "$FIRST_RUN" ]; then
    TAR_FILES="$TAR_FILES $FIRST_RUN"
fi
tar -czvf dist/archive.tar.gz $TAR_FILES
