#!/bin/bash

# PNGファイルをEPSファイルに変換するスクリプト
# ImageMagickが必要です

# Figディレクトリのパス
FIG_DIR="/home/initial/lab_ws/paper_ws/src/proto/product_v2/Fig"

# Figディレクトリに移動
cd "$FIG_DIR"

# すべてのPNGファイルをループ処理
for png_file in *.png; do
    # ファイル名から拡張子を除いた部分を取得
    base_name="${png_file%.png}"
    
    # 既にEPSファイルが存在する場合はスキップ
    if [ -f "${base_name}.eps" ]; then
        echo "既に存在します: ${base_name}.eps - スキップします"
        continue
    fi
    
    echo "変換中: $png_file -> ${base_name}.eps"
    
    # PNGをEPSに変換
    convert "$png_file" "${base_name}.eps"
    
    # 変換が成功したか確認
    if [ $? -eq 0 ]; then
        echo "変換成功: ${base_name}.eps"
    else
        echo "変換失敗: $png_file"
    fi
done

echo "変換完了"
