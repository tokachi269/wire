# Roadの対応範囲

この文書は、Roadの明示的なcommit試行について対応範囲を定める。pointer移動は軽量guideだけを更新し、
commitとして分類しない。

## 通常描画で対応する構成

道路は登録済みの断面を使って描画する。Coreは断面catalogueを内蔵しない。利用者が選択できる断面一覧は
`web/src/road_templates.ts`が所有し、新規workspaceは`AddRoadLayoutTemplate`で登録して、Coreが返したIDを保持する。
保存済みworkspaceを開き直す場合はarchive内の断面を使い、catalogueを再登録しない。

次は通常対応する構成であり、製品要件を理由に拒否してはならない。単独の直線道路またはBezier道路、
独立した道路の反復作成、degree 2のpass-throughまたはcorner、T字・十字を含む対応済みdegree 3/4 junction。
endpoint延長は明示されたendpoint IDを使い、segmentへの分岐は明示されたsegment IDとsegment内距離を使う。

回帰testは、直線・Bezierの反復session、Enter、Escape、tool切替、commit拒否後の再試行、endpoint延長、
T字・十字生成、斜めjunction、実際のWASM scene更新を対象とする。

## 製品要件による制約

`RequirementConstraint`は文書化された製品規則だけに使う。現在Roadにある規則は、新しく送られたPathが
自己交差してはならないことだけである(`road_path_self_intersection`)。固定角度範囲、resolverの対応不足、
geometry実装上の制限は製品要件による制約ではない。

## failureのowner

| owner | 入力条件 | category | 通常描画への影響 |
|---|---|---|---|
| operation preflight (`operations/`) | ID欠落、非有限入力、長さ0のspan、不正なrequest | `InvalidInput` | ClickまたはEnterの後だけ表示する |
| operation preflight (`operations/`) | 文書化されたPath自己交差 | `RequirementConstraint` | 利用者が入力Pathを変更する必要がある |
| connection解決 (`generation/connections.cpp`) | 正しいtopologyだが現在のresolver対応外、対応可能なsetback不足、未対応の断面組合せ | `NotImplemented` | 利用者の入力エラーではなく、bugまたは対応範囲backlog |
| segment・section・marking解決 | 正しい操作だがtransitionまたはmarkingのmerge/splitが未実装 | `NotImplemented` | 操作はatomicかつ再試行可能なまま |
| generationとgeometry | 正しいauthoritative参照または必須のderived値が欠落 | `InternalError` | 対応済み経路の不具合 |
| persistence | fieldの重複・欠落・不明、無効なID、非有限値 | `InvalidInput` | stateを置換する前にloadを拒否する |
| draw session | 明示requestがanchor確定後に変更されたstateを参照 | `StateConflict` | anchorを更新して再試行する。現在の同期Road経路はこのcategoryを返さない |

commit失敗は必ず空でないreason codeを持つ。UIは`NotImplemented`を不正入力ではなく未実装として表示する。
内部invariant違反を`RequirementConstraint`へ変換してはならない。

## 未対応のfamily

現在の実装制限には、断面transition区間内でのsplit、実装済み断面組合せ外のconnection layout、
未解決のmarking merge/splitがある。意味上のoperationとgeometry resolverが実装されるまでは
`NotImplemented`とする。最寄りのapproach、最寄りのboundary、fallback shapeは選択しない。

ratio指定の断面transitionより前または後でのsplitは対応済みで、transitionの`t`を保持側segmentへ
再正規化する。transition区間内のsplitは具体的な理由で拒否し、stateを変更しない。

boundaryは断面profileを持つため、L字溝、curb、縁石付きmedian edgeのように固有の面を持つ端部構造も
対応する断面である。profileはlayout幅を消費せず、両隣のstrip内へ張り出す。そのため、2.0mと宣言した
walkwayは、gutterが先頭0.1mへ張り出しても2.0mのままである。profileの測定基準は`architecture.md`で定める。

生成できないprofileはstate変更前に拒否する。点が逆行するもの、隣接stripを越えて張り出すもの、
face数とpoint数が一致しないもの、存在しないsurface styleを指定するものが該当する。
最寄りの生成可能な形状へ置き換えない。

profileには制限が1つ残る。profileは2つのstrip間にしか置けないため、外側に何もない道路、つまりその側に
walkwayがない道路のgutterは宣言場所を持たない。degree 2 connectionとjunctionは同じsemantic side mappingを
使う。両carriageway edgeと両road outer edgeを固定し、追加のinner faceは幅0から開始する。
欠落profileの代わりにgutter自体を使わない。laneまたはboundaryの継続が曖昧なら明示topologyを要求し、
引き続き`NotImplemented`とする。

1回で描くintervalは2本の既存道路を接続できる。両端は既存nodeまたは既存segment上の明示距離を指定し、
両segmentのsplitと接続segmentをatomicにcommitする。同じsource segment上の2点を接続する操作は未対応である。

Lane BranchとMergeは利用できない。旧editorはraw boundary IDを利用者へ選ばせ、defaultも自動選択していたため、
非表示にするのではなくoperation自体を削除した。junction内のlane connectionは引き続きCoreが導出する。
branchとmergeを再開する条件は`backlog.md`に記載する。

## sessionの動作

ClickとEnterは明示的なdraw action結果を返す。commitが拒否されてもanchorとguideを維持する。
pointer移動で直前のcommit failureを上書きしない。anchorはあるがguideがない状態のEnterはtransient sessionを
明示的に終了し、active sessionがない状態のEnterは`ignored/session-inactive`を返す。
