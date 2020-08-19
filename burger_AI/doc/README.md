# 概要

# 依存関係
burger_warのパッケージに比べて追加するもの

# フォルダ構造

burger_AI
├─ doc
│   │   各種説明ファイル
│   │
│   └── README.md (このファイル)
│
├─ bash_scripts
│   │   burger_AIの学習実行用の .shファイル
│   │   
│   ├── setup_sim.sh    : シミュレーションのセットアップ
│   ├── reset.sh        : シミュレーション状態のリセット
│   └── start.sh        : 走行開始
│
├─ scripts
│   │   ROSのノードとか
│   │
│   └── hogehoge
│
├─ launch
│   │   launchファイル
│   │
　   └── setup.launch    : 走行環境のセットアップ(自機のみ)


