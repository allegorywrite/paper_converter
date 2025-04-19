#!/bin/bash

# 監視対象のTeXファイル
TEX_FILE="temp_en_modified.tex"
DIR="/home/initial/lab_ws/paper_ws/src/proto/product_temp"

# カレントディレクトリを設定
cd "$DIR"   

echo "自動コンパイルを開始します。$TEX_FILE の変更を監視しています..."
echo "終了するには Ctrl+C を押してください。"

# 初回コンパイル
echo "初回コンパイルを実行します..."
platex -interaction=nonstopmode "$TEX_FILE" || echo "platexでエラーが発生しましたが、処理を続行します。"
dvipdfmx "${TEX_FILE%.tex}.dvi" || echo "dvipdfmxでエラーが発生しましたが、処理を続行します。"
echo "コンパイル完了: ${TEX_FILE%.tex}.pdf が生成されました。"
