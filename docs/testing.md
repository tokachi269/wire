# 検証方針

この文書はrepository全体で共有する、project非依存の検証方針の正本である。Architectureの意味論は`docs/architecture.md`と各domain architecture文書、操作の意味は操作意味論文書を正本とする。Test、ledger、manifest、coverage reportはevidenceであり、意味論の正本ではない。

Project固有の仕組みは別文書で定義する。このrepositoryでは[`wire/testing.md`](wire/testing.md)と[`road/testing.md`](road/testing.md)を参照する。

## 契約を中心とした検証

管理単位はtest case数ではなく契約である。重要な契約は通常、次のevidenceを持つ。

- Primary proof: 契約を最も直接的に証明する1件
- Secondary proof: 異なるriskを検出できる場合に限る、1〜2件のproductionに近い別経路
- Structural guard: 振る舞いだけでは構造を証明できない場合のdependency、type、compile/link、module、安定したsource境界

Primary proofの欠落を、多数の間接scenarioやSourceGuardで代用しない。同じfaultへcaseを追加する前に、既存evidenceの強化・置換・退役を検討する。

## 保護手段の優先順位

対象のfault classを防げる最も強い仕組みを優先する。

```text
capability / type / module boundary
  > architecture structural lint
  > behavioral Primary proof
  > metamorphic / differential proof
  > Scenario / End-to-end proof
  > full regression suite
```

上位の仕組みが下位の仕組みを常に不要にするという意味ではない。異なるfault classを検出する仕組みは残す。品質はtest数やcoverage率ではなく、各重要契約に独立した証明があるかで評価する。

## Evidenceの分類

| 分類 | 証明するもの |
|---|---|
| Structural | dependency、type、compile/link境界、安定したsource architecture |
| Invariant / Contract | 正本状態と意味上の不変条件 |
| Metamorphic | 入力反転、操作順、表現変更をまたいだ意味の保存 |
| Differential | scoped/fullまたは等価なentry path間の一致 |
| Scenario | 重要incidentの再現またはproductionに近い構成 |
| End-to-end | adapter、transport、runtime、user-facing境界 |

SourceGuardはStructural evidenceであり、振る舞いの証明を代替しない。

## Oracleの独立性

Primary proofの期待値を、検査対象と同じproduction decisionやhelperを呼んで計算してはいけない。それで証明できるのは自己整合だけであり、正しさではない。独立した式またはoracle、既知のanchor、metamorphic relation、別実装・別経路とのdifferentialを使う。Production helperの再利用は、独立したPrimary proofがある場合のSecondary consistency assertionとしては利用できる。

## 振る舞いprobe catalog

Systemの性質と現実的なfault modelからprobeを選ぶ。このcatalogは必須checklistでもtest数quotaでもない。

| Systemの性質 | 代表的なprobe |
|---|---|
| Stateful mutation | failure atomicity、無関係なstateへの非干渉、操作が保証する場合の決定的repeat |
| Persistence | save → load → save、load後のderived rebuild equivalence、失敗したloadでidentity・ID・counterが不変 |
| Incremental / cache結果 | incremental結果とclean/full rebuildのdifferential、update後のstale cache検出 |
| Stable identity | array reorderでidentity不変、geometry変更でidentity不変、missing identityを近接・名前・順序から推測せず拒否 |
| 独立したcomposition | A単独とA+B内のAを比較、独立componentの順序をpermutation |
| 等価なentry | API、adapter、UI、transport entry間で意味上のrequest/resultを比較 |
| Geometry / directional data | 定義された意味を保つinput reverse、symmetry、permutation |
| Stateful sequence | seed固定の決定的操作列、各stepのinvariant、定期的persistence roundtrip、invalid操作でstate不変 |

### Pattern選択guide

| Featureの性質 | 最初に検討するもの |
|---|---|
| authoritative mutation | atomicityとnon-interference |
| cache / incremental derivation | full-vs-incremental differentialとstale-cache probe |
| stable identity | reorderとgeometry-changeのmetamorphic probe |
| persistence | roundtrip、derived rebuild、failed-load時のcounter安定性 |
| 複数adapter / entry | semantic entry differential |
| 順序非依存のcomposition | component permutationとA-vs-A+B |
| directional geometry | reverse、symmetry、定義済みpermutation relation |
| multi-step state machine | seeded sequence、stepごとのinvariant、定期roundtrip、invalid stepで不変 |

実在する契約と現実的なfaultに対応するprobeだけを選ぶ。独立したoracleを持つ場合は、1つの性質に複数probeが必要なこともある。

## Fail-firstと診断

新scenarioまたはbug fixでは、現行production behaviorが契約に違反することを最初に示す。すでにpassするcaseを追加するだけではregression proofにならない。Assertionは説明のないboolではなく、最初に失敗したoperation境界またはinvariantを示す。

挙動変更ゼロを主張する変更では、authoritative byte equality、bit equality、既存behavioral testの無変更通過など、契約に適した等価性の証明方法を明示する。Skipされたtestはpassではない。

## Testの有効性

Line/branch coverageが示すのは実行でありfault検出ではない。小さく代表的なsemantic faultを注入し、意味のある契約境界でfailするかによりverification familyを評価する。構文的mutationは何も証明しないassertionの検出には有効だが、state ownership、stale derived output、partial mutation、transport drift、silent fallback failureまでは覆わない。

過去incidentは価値の高いfault modelである。実用的ならold commitをbuildする代わりに同じsemantic faultを現行codeへ再現する。Mutationとsemantic fault injectionは一時的なverification auditであり、通常は恒久testとしてcommitしない。有用な調査履歴は現行契約仕様ではなくaudit logへ残す。

Green suiteからregressionが漏れた場合は、missing invariant、production fidelity不足、missing scenario、weak semantic assertion、condition complexity、implementation-detail assertion、旧fixが1 fixtureだけに閉じ込められている、のどれかを分類する。症状caseだけを追加せず、契約ownerを強化する。

## Regression lifecycle

Bug fixでは次のlifecycleを使う。

```text
incident reproducer
  -> root cause
  -> underlying contract
  -> Primary proofの作成または強化
  -> semantic faultの再注入
  -> 強化したproofが検出することを確認
  -> fixを実装
  -> counterfactualと無関係stateを確認
  -> 完全に包含されたreproducerを削除または縮小
```

Bugごとに恒久testを1件残すことを既定にしない。その入力またはoracleが基礎契約の直接証明とは別の価値を持つ場合だけincident固有Scenarioを残す。

Production fixで新しいfailureが出た場合、まず直前の変更を縮小またはrevertできるか検討する。補償behaviorの追加を第一選択にしない。

## Architecture structural guard

Project architecture manifestはscan root、source extension、exclude、exactly-one-layer pattern、layerごとのforbidden tokenを定義できる。Structural lintはdependencyとownership境界を保護するが、projectのproduction semanticsをmanifestへ複写したり、token scanだけで意味上の正しさを主張したりしてはいけない。

より強いtype system、compile境界、module dependency、capability、architecture lintによりinvalid stateを表現不能にできた場合、source grep/regex guardは退役させる。廃止済み実装形状を恒久的に固定しない。

## Testの退役

既存testをverification consolidationとして削除できるのは、次をすべて満たす場合だけである。

1. 契約が特定されている。
2. 別のPrimary proofが存在する。
3. 旧testが検出したfaultを再現できる。
4. Primary proofがそのfaultを検出する。
5. 旧testが独立したoracleを持たない。

条件を証明できない疑似duplicateは残す。削除数を進捗にしない。

## Full suite

Focused proofはroot causeと直接の契約を証明する。Full suiteはcross-domain effect、registration gap、transport、既存契約との競合を調べるSecondary safety netであり、fail-first evidenceやfocused Primary proofの代替ではない。

完了には、要求されたbehavior、そのarchitecture contract、focused proof、関連structural check、説明不能な補償がないこと、scope containmentが必要である。Full suiteがgreenであるだけでは不十分である。
