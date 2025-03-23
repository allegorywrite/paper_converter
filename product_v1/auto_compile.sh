#!/bin/bash

# 監視対象のTeXファイル
TEX_FILE="ISCS_Murakami_forFinal_v1.1.tex"
DIR="/home/initial/lab_ws/paper_ws/src/proto/product_v1"

# カレントディレクトリを設定
cd "$DIR"

echo "自動コンパイルを開始します。$TEX_FILE の変更を監視しています..."
echo "終了するには Ctrl+C を押してください。"

# 初回コンパイル
echo "初回コンパイルを実行します..."
platex "$TEX_FILE"
dvipdfmx "${TEX_FILE%.tex}.dvi"
echo "コンパイル完了: ${TEX_FILE%.tex}.pdf が生成されました。"

# 変更を監視して自動コンパイル
while true; do
    inotifywait -e modify "$TEX_FILE"
    echo "ファイルの変更を検出しました。コンパイルを実行します..."
    platex "$TEX_FILE"
    dvipdfmx "${TEX_FILE%.tex}.dvi"
    echo "コンパイル完了: ${TEX_FILE%.tex}.pdf が生成されました。"
done
