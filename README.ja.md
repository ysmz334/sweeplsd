# SweepLSD

**One sweep, all segments. — ひと掃きで、すべての線分を。**

SweepLSD は画像を上から下へ**ちょうど1回だけ走査**して全線分を検出する線分検出器です。
画素処理コアは整数演算のみ、保持するのは数行分の状態だけ（**O(width) メモリ**）—
FPGA向きの設計で、CPU上でも非常に高速です。

*（2014年の修士論文（清水義泰, 早稲田大学）で「OPLSD」として提案された手法の、
ゼロからの C++17 再実装＋計測済み改善版です。）*

![1回のラスタ走査: 走査線が最終行を通過した瞬間に線分が確定する](assets/sweep.gif)

*実際にこう動きます: 上から下への1回の走査、保持するのは数行分の状態だけ。
各線分は最後の画素が通過した瞬間に確定します（ラベリングは走査より7行遅れて追走）。*

## 主要数値

FullHD（1920×1080）グレースケール写真、i7-8700K、AVX2、シングルスレッド。
比較対象はすべて**原著者の本家実装**（von Gioi の LSD、Akinlar & Topal の ED_Lib
EDLines）を同一ISAターゲットでビルドしたものです。

| | SweepLSD (one-pass) | EDLines (ED_Lib) | LSD |
|---|---|---|---|
| 1枚あたり中央値時間 | **約17 ms** | 約43 ms | 約230 ms |
| 中間バッファのメモリ | **O(width)** | O(pixels) | O(pixels) |
| 線分方向誤差（合成GT） | **0.01–0.04°** | 0.14° | 0.14° |
| F-max クリーン〜低ノイズ (σ0–5) | **0.963–0.969** | 0.953–0.954 | 0.92 |
| F-max 強ノイズ (σ10–20) | 0.907–0.949 | **0.954–0.961** | 0.50–0.80 |

制約も計測して明記しています: SweepLSD のコントラスト・ゲート型エッジモデルは、
LSD/EDLines（方向整合性ベースの連結）が拾える低コントラストの緩いエッジを取り逃します。
消失点推定ではこの差は推定器の選択で補償できます — `docs/vp_evaluation.html` 参照。

## クイックスタート

```cpp
#include <sweeplsd/sweeplsd.hpp>
#include <sweeplsd/io.hpp>

int main() {
    sweeplsd::GrayImage img = sweeplsd::loadGray("photo.png");
    auto segments = sweeplsd::detect(img, sweeplsd::Params::improved());
    // segments[i].x0 / y0 / x1 / y1（サブピクセル端点）
    sweeplsd::saveSegmentVisualization("out.png", img, segments);
}
```

ビルド（GCC / Clang / MSVC いずれも、CMake ≥ 3.15）:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build            # パリティテスト: one-pass == multi-pass
./build/sweeplsd_cli photo.png out.png
```

CMake パッケージとして: `find_package(sweeplsd CONFIG REQUIRED)` の後、
`sweeplsd::sweeplsd`（コア、依存ゼロ）、`sweeplsd::io`（stb による PNG/JPG）、
`sweeplsd::opencv`（ヘッダオンリー `cv::Mat` アダプタ、OpenCV の LSD の置き換え）を
リンクしてください。

**コンパイラ性能の注記。** カーネルは設計方針として SIMD イントリンシクスを含みません
（FPGA志向・可読性重視）。速度はコンパイラの自動ベクトル化によるもので、GCC / Clang は
完全にベクトル化します（FullHD 約15〜18ms）。MSVC はビルド・全テスト通過しますが、
現状バイトカーネルをベクトル化しないため約53ms（約3倍遅）です。Windows で性能が必要な
場合は MinGW-w64 か clang-cl を推奨します。

## 仕組み

ラスタ走査上の行ストリームとして表現された5段:

1. **ガウシアン** 5×5 分離可能ぼかし（整数、`>>10` 一回）
2. **勾配** 2×2 オペレータ; パワー `(|dx|+|dy|+1)/2`、方向は水平/垂直に量子化
3. **エッジ** 閾値＋勾配方向の非極大抑制
4. **端点候補** 5×5 リング論理で線分の開始/終端/分岐を検出
5. **ラベリング＋判定** 散布モーメントを担いだストリーミング連結成分;
   画素数と PCA 固有値比（細く真っ直ぐか）で線分判定

同一カーネルを共有する2つのドライバ（出力一致をテストで保証）:
`detect()`（段ごとに全画像1パス — 最も読みやすい）と
`detectOnePass()`（ストリーミング単一走査 — O(width) メモリで最速）。
`Params{}` は2014年論文を忠実に再現、`Params::improved()` は計測済み改善
（サブピクセルNMS、ストリーミングヒステリシス、曲線除去、半画素格子補正など）を
有効化します。

段ごとの図付き解説は `docs/`（GitHub Pages）にあります。

## サンプル

- `examples/manhattan_frame.cpp` — **較正済みManhattan枠／消失方向**。SweepLSD に
  最適と計測された推定器構成（線分1本1票・垂直事前シード・強候補探索）:
  `sweeplsd_manhattan photo.jpg --focal 675`
- `examples/vanishing_points.cpp` — 非較正の逐次RANSAC消失点（内部パラメータ不要）
- `examples/opencv_detect.cpp` — ヘッダオンリーアダプタ経由で OpenCV コードから利用

## このプロジェクトについて

SweepLSD の設計は清水義泰（2014年修士論文、国内学会で OPLSD として発表。論文自体は
本リポジトリでは配布していません）。本再実装・改善・評価スイートは、著者が
Claude（Anthropic）との AI 協働ペア作業で構築したもので、アルゴリズム上の主張は
すべて `docs/` の計測で検証されています。

## ライセンス

MIT — [LICENSE](LICENSE) 参照。`third_party/stb/` に stb_image / stb_image_write
（パブリックドメイン/MIT）を同梱しています。
