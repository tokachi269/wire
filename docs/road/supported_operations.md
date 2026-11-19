# Roadの対応範囲

この文書は、Roadで明示的に確定操作を試みる場合の対応範囲を定める。ポインターの移動では軽量ガイドだけを更新し、確定操作として分類しない。

## 通常描画で対応する構成

道路は登録済みの断面を使って描画する。Coreは断面のcatalogを内蔵しない。利用者が選択できる断面一覧は`web/src/road_templates.ts`が所有し、新規workspaceは`AddRoadLayoutTemplate`で登録して、Coreが返したIDを保持する。

保存済みworkspaceを開き直す場合はarchive内の断面を使い、catalogを再登録しない。

次は通常対応する構成であり、製品要件を理由に拒否してはならない。単独の直線道路またはBezier道路、独立した道路の反復作成、degree 2のpass-throughまたはcorner、T字・十字を含む対応済みdegree 3/4 junction。endpoint延長は明示されたendpoint IDを使い、segmentへの分岐は明示されたsegment IDとsegment内距離を使う。

回帰テストは、直線・Bezier道路を繰り返し描くsession、Enter、Escape、toolの切り替え、確定拒否後の再試行、endpoint延長、T字・十字生成、斜めjunction、実際のWASMによるsceneの更新を対象とする。

## 製品要件による制約

`RequirementConstraint`は文書化された製品規則だけに使う。現在Roadにある規則は、新しく送られたPathが自己交差してはならないことだけである(`road_path_self_intersection`)。固定角度範囲、resolverの対応不足、幾何実装上の制限は製品要件による制約ではない。

## 失敗時の責務

| 担当箇所 | 入力条件 | category | 通常描画への影響 |
|---|---|---|---|
| 操作preflight (`operations/`) | ID欠落、非有限入力、長さ0のspan、不正なrequest | `InvalidInput` | ClickまたはEnterの後だけ表示する |
| 操作preflight (`operations/`) | 文書化されたPath自己交差 | `RequirementConstraint` | 利用者が入力Pathを変更する必要がある |
| connection解決 (`generation/connections.cpp`) | 正しいtopologyだが現在のresolver対応外、対応可能なsetback不足、未対応の断面組合せ | `NotImplemented` | 利用者の入力エラーではなく、不具合または対応範囲の保留事項 |
| segment・section・marking解決 | 正しい操作だがtransitionまたはmarkingのmerge/splitが未実装 | `NotImplemented` | 操作の原子性と再試行可能な状態を維持する |
| generationと幾何 | 正しいauthoritative参照または必須のderived値が欠落 | `InternalError` | 対応済み経路の不具合 |
| persistence | フィールドの重複・欠落・不明、無効なID、非有限値 | `InvalidInput` | 状態を置換する前にloadを拒否する |
| draw session | 明示requestがanchor確定後に変更された状態を参照 | `StateConflict` | anchorを更新して再試行する。現在の同期的なRoadの処理経路はこの分類を返さない |

確定操作の失敗には、必ず空でない理由コードを付ける。UIは`NotImplemented`を未実装として表示し、不正入力として扱わない。内部不変条件違反を`RequirementConstraint`へ変換してはならない。

## 未対応のfamily

現在の実装制限には、断面transition区間内でのsplit、実装済み断面組合せ外のconnection layout、未解決のmarking merge/splitがある。意味上の操作と幾何resolverが実装されるまでは`NotImplemented`とする。最寄りのapproach、最寄りの境界、fallback shapeは選択しない。

ratio指定の断面transitionより前または後でのsplitは対応済みで、transitionの`t`を保持側segmentへ再正規化する。transition区間内のsplitは具体的な理由で拒否し、状態を変更しない。

境界は断面profileを持つため、L字溝、curb、縁石付きmedian edgeのように固有の面を持つ端部構造も対応する断面である。profileはlayout幅を消費せず、両隣のstrip内へ張り出す。そのため、2.0mと宣言したwalkwayは、gutterが先頭0.1mへ張り出しても2.0mのままである。

profileの測定基準は`architecture.md`で定める。

生成できないprofileは状態変更前に拒否する。点が逆行するもの、隣接stripを越えて張り出すもの、face数とpoint数が一致しないもの、存在しないsurface styleを指定するものが該当する。最寄りの生成可能な形状へ置き換えない。

profileには制限が1つ残る。profileは2つのstrip間にしか置けないため、外側に何もない道路、つまりその側にwalkwayがない道路のgutterは宣言場所を持たない。degree 2 connectionとjunctionは同じsemantic side mappingを使う。

両carriageway edgeと両road outer edgeを固定し、追加のinner faceは幅0から開始する。欠落profileの代わりにgutter自体を使わない。laneまたは境界の継続が曖昧なら明示topologyを要求し、引き続き`NotImplemented`とする。

1回で描くintervalは2本の既存道路を接続できる。両端は既存nodeまたは既存segment上の明示距離を指定し、両segmentのsplitと接続segmentをatomicにcommitする。同じsource segment上の2点を接続する操作は未対応である。

Lane BranchとMergeは利用できない。旧editorはraw境界IDを利用者へ選ばせ、defaultも自動選択していたため、操作自体を削除した。非表示にするだけの対応は行っていない。junction内のlane connectionは引き続きCoreが導出する。branchとmergeを再開する条件は`backlog.md`に記載する。

## sessionの動作

ClickとEnterは、描画操作の結果を明示して返す。確定を拒否した場合もanchorとガイドを維持する。ポインター移動で直前の確定失敗を上書きしない。anchorはあるがガイドがない状態でEnterを押すと、一時的な描画sessionを明示的に終了する。有効なsessionがない状態でEnterを押すと、`ignored/session-inactive`を返す。
