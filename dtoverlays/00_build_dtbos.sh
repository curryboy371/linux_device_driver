#!/bin/bash

# 오류 발생 시 종료
set -e

# 현재 디렉토리 내 모든 .dts 파일 처리
for DTS_FILE in *.dts; do
    # 확장자 제거한 기본 이름 추출
    BASE_NAME="${DTS_FILE%.dts}"
    DTBO_FILE="${BASE_NAME}.dtbo"

    echo "[*] Compiling $DTS_FILE -> $DTBO_FILE ..."
    dtc -@ -I dts -O dtb -o "$DTBO_FILE" "$DTS_FILE"

    echo "[*] Copying $DTBO_FILE to /boot/overlays/ ..."
    sudo cp "$DTBO_FILE" /boot/overlays/

    echo "[✓] Done: $DTS_FILE"
done

echo "[✔] All .dts files compiled and copied successfully."
