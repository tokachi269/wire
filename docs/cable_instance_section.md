# CableInstance / CableSection

`Span` は topology 上の区間であり、描画上の cable identity ではない。`CableInstance` と
`CableSection` は visual derive 層の概念であり、`SavedBackboneGraph`、Span、Bundle、Port の
正本を決めない。

## population section

population section は、同じ `logical_span_id` に属する span visual assembly member である。
base section と population sections はそれぞれ既存の endpoint、sag、curve policy から通常の
body curve を作る。population は `SavedBackboneGraph`、Span、Port を増やさず、配置不能時は
omit diagnostic を残す。

## span visual assembly

helix は CableSection ではなく、logical span 単位で派生する identity を持たない visual part
である。member の関連は `CableSectionKey.logical_span_id` による明示キーだけで解決し、別の
CableSection を carrier として保存する関係や geometry 近傍探索は持たない。

assembly は base section、同じ logical span の population sections、support path、helix を一時的に
まとめる。Bundle の複数 lane を一つの束として扱わず、lane ごとの logical span に独立して適用する。
helix と support path は topology、connectivity、CableRun identity を変更しない。
support path はhelixなしでも生成できる。全support pathは同じprimary curve生成を使う。
support bandが0なら解決済みmember endpointへtrim区間で収束し、中央部だけ線径分離する。
正数bandなら明示band endpointを使う。別laneを一つのsupport pathへまとめない。
helixはband 0でも有効であり、既定OPTICALはmember endpoint追従を使う。

## 禁止する実装

- 追加線や helix を `SavedBackboneGraph` span として保存する
- `CableSection` を第二の topology 正本にする
- geometry proximity から member、carrier、support path を推測する
- viewer 側で member grouping や continuity を補正する
- `CableMember`、`CableMemberGroup`、`WrapCable`、`SupportWire` を新しい中心型として増やす
