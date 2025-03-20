# 単調環境におけるUAVの協調自己位置推定論文プロジェクト

このリポジトリは、「Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filter」（単調環境におけるUAVの協調自己位置推定）に関する論文プロジェクトを含んでいます。

## プロジェクト概要

本プロジェクトでは、日本語で書かれた既存の論文「Stein Particle Filterを用いた単調環境におけるUAVの協調自己位置推定」をトップカンファレンスに通せるように推敲し、英語論文に変換しました。

論文の主な内容：
- 単調環境（農地や森林など）におけるUAVの協調自己位置推定の課題
- Stein Particle FilterとRelaxed ADMMを組み合わせた新しいフレームワークの提案
- 階層型尤度の導入による特徴マッチングの改善
- シミュレーションと実機実験による検証

## リポジトリ構成

```
.
├── README.md                 # このファイル
├── policy.md                 # 論文執筆ポリシー
├── console.md                # 執筆者のみが答えられる質問の記録
├── material/                 # 元の資料
│   ├── 60QA.docx             # 論文の60の質問と回答
│   ├── MSCS2025_Arita_TN_rev1.pdf  # 元の日本語論文
│   └── MSCS2025_Arita.pptx   # プレゼン資料
├── misc/                     # OCR結果など
│   ├── 60QA_text.md          # 60QAのテキスト化
│   ├── MSCS2025_Arita_ocr_result.md  # プレゼンのOCR結果
│   ├── MSCS2025_Arita_text.md        # プレゼンのテキスト
│   └── MSCS2025_Arita_TN_rev1_ocr_result.md  # 論文のOCR結果
├── product_v1/               # 英語論文の初期バージョン
├── product_v2/               # 英語論文の最終バージョン
│   ├── ISCS_Arita_forFinal_v1.0.tex  # 最終論文のTeXファイル
│   ├── ISCS_Arita_forFinal_v1.0.pdf  # 最終論文のPDFファイル
│   ├── auto_compile.sh       # コンパイルスクリプト
│   ├── SICE_ISCS.cls         # 論文クラスファイル
│   └── Fig/                  # 図表ファイル
├── product_temp/             # 論文テンプレート
└── utils/                    # ユーティリティスクリプト
    ├── extract_docx.py       # DOCXファイルからテキスト抽出
    ├── extract_pptx.py       # PPTXファイルからテキスト抽出
    ├── pptx_to_pdf.py        # PPTXをPDFに変換
    └── run_mistral_ocr.py    # OCR処理スクリプト
```

## 論文執筆プロセス

1. 元の日本語論文（MSCS2025_Arita_TN_rev1.pdf）とプレゼン資料（MSCS2025_Arita.pptx）をOCRしてテキスト化
2. 概要部分を60QAに流し込み、論文の主要な質問に回答
3. 60QAで答えられていない部分をサーベイに投げて補完
4. 60QAと元原稿をまとめて英語原稿に書き換え
5. LaTeXで執筆し、フォーマットを整え、コンパイルして検証

## 論文コンパイル方法

最終論文をコンパイルするには、以下のコマンドを実行します：

```bash
cd product_v2
./auto_compile.sh
```

これにより、ISCS_Arita_forFinal_v1.0.pdfが生成されます。

## 主な成果

- 日本語論文を英語に翻訳し、トップカンファレンスの基準に合わせて推敲
- 関連研究のより包括的なレビューの追加
- 問題設定の明確化と背景の詳細な説明
- 提案手法の数学的定式化の洗練
- 実験結果の詳細な分析と考察
- 将来の研究方向の明確化

## 注意事項

- 論文内の図表は元の日本語論文から適切に変換・修正されています
- 参考文献は適切に引用されています
- LaTeXコンパイルにはpdflatexを使用しています
