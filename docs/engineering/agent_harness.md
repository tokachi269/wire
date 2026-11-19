# 移植可能なAgent Engineering Harness

この文書では、エージェントによる開発支援に共通する作業手順を定める。責務の境界と検証の進め方を扱い、製品の仕様は各プロジェクトの設計文書に記載する。

## 新しい概念を導入する手順

実装前に、すでに決まっている判断だけを記録する。

```text
Decision
  -> Owner
  -> Inputs
  -> Authoritative output
  -> downstream consumers
  -> failure owner
  -> transaction boundary
```

1つの判断を担当する箇所は1つに限定する。利用側は、同じ入力から判断を繰り返さず、決定済みの正本出力を使う。まだ存在しない概念のために抽象化を設計しない。

各契約の保護手段は次の順で選ぶ。

1. Invalid状態を構造的に表現不能にできるか。
2. Type、module、capabilityで防げるか。
3. 安定した境界をarch lintで保護できるか。
4. Behavioral Primary proofが必要か。
5. 意味のあるmetamorphic relationがあるか。
6. Differential proofに使える等価経路があるか。
7. User-facing ScenarioまたはEnd-to-end proofが必要か。

故障の種類によっては、複数の段階で証拠を揃える必要がある。Token scanでは動作の正しさを、E2Eシナリオではdependency ownershipを証明できない。Behavioral probeは`docs/testing.md`の振る舞いprobe一覧から選び、項目数の達成を目標にはしない。

## Guardrail昇格の段階

防御を無制限に積み上げない。基礎契約が明確になったら次の順に昇格させる。

```text
incident-specific assertion
  -> generalized behavioral contract
  -> architecture lint
  -> type / module / capability boundary
```

昇格後は元の故障を再注入する。強い仕組みがそれを検出または防止でき、弱いguardに独立した期待値の根拠がない場合は、そのテストまたはguardを退役候補として評価する。

## バグ修正手順

```text
fail-first
  -> incident reproducer
  -> root cause
  -> underlying contract
  -> stronger Primary proof
  -> semantic fault injection
  -> minimal causal fix
  -> counterfactual and unrelated-state check
  -> reproducer retirement evaluation
```

製品コードの各変更について、根本原因、変更が必要な理由、その変更がなければ失敗する対象を絞った検証を説明する。修正後に別の失敗が出た場合は、補償処理を加える前に、直前の変更を縮小またはrevertできるか調べる。実装を通すために既存契約を弱めない。

## Architecture-significant change workflow

正本や保存状態、公開APIやCore API、操作意味論、decision owner、モジュールや層の依存関係、層やドメインをまたぐ依存関係、fallbackや特別な経路、永続化を変更する場合は、architecture-significantとして扱う。

実際の変更が想定範囲を超えた場合も、途中からこの手順へ切り替える。小さな不具合や局所変更の全てに機械的に適用する必要はない。

設計上の問題、繰り返す不具合、新しいguardrailを検討する問題、既存の分類が分からない問題では、実装前にSWEBOK v4を主な分類体系として位置付けを調べる。これは開発手順内の調査であり、分類器、プロジェクト独自の分類体系、恒久的な台帳は作らない。

複数のKnowledge Areaを選んでよい。Systems Engineeringまで範囲が広がる場合だけ、SEBoKを補助に使う。

```text
Problem:
Observed symptom:
Likely SWEBOK KA:
Established terminology:
Known techniques / literature:
Project-specific gap:
Need for a new Wire-specific mechanism:
```

既存技法で不足を解消できる場合は、名前だけを変えたframeworkを作らない。Wire固有の仕組みが必要な理由を書く欄には、既存技法、既存harness、既存の正本では満たせない部分だけを記述する。

実装前にexpected impactを記録する。

```text
Goal:
Problem classification:
Expected owners:
Expected modules:
Expected state/schema impact:
Expected dependency impact:
Boundaries expected not to change:
Known architectural risks:
```

実装後は同じ作業単位でactual impactとArchitecture Deltaを比較する。

```text
Actual owners:
Actual modules:
Actual state/schema impact:
Actual dependency impact:
Architecture Delta:
Unexpected expansion:
Explanation:
```

workflowは次の順序とする。

```text
problem classification
  -> expected impact
  -> implementation
  -> actual impact
  -> Architecture Delta
  -> regression proof
```

想定と実際の変更が大きく異なる場合は、補償実装を続けず、原因と作業範囲を評価する。特に、第二のauthoritative representation、authoritative/derived間での意味の二重化、persisted mirror/cache、generic layerへの機能固有概念の流入、guardを一時解除して既存操作を別経路から呼ぶこと、操作固有の動作が下流のmaterialization/viewerへ入り込むこと、新しいfallback inference、想定外の箇所への説明できない変更の波及があれば、作業を止めて評価する。

Tokenやpathの検査で示せるのは、レビューが必要な候補までである。それだけで、これらの設計上の意味を証明することはできない。

完了時は次の三方向からレビューする。

- Backward: 既存製品の仕様、architecture契約、テスト契約を壊していないか。
- Inward: 設計上、実際に何を変えたか。想定と実際の変更の差を説明できるか。
- Forward: 次の変更を不必要に難しくするauthority、dependency、special pathを増やしていないか。完全な自動判定は主張しない。

Architecture observationのDSM、Reflexion Model、Architecture Delta、co-change、hotspotは、レビューのための証拠として使う。数値そのものを失敗の閾値にしない。自動的に失敗とするのは、既存のarch lintなどで客観的に判定できる契約違反に限る。

より強い仕組みに役割を吸収されたもの、他の観測手段と同じ情報しか返さないもの、有用な兆候が得られないもの、維持コストがレビュー上の価値を上回るものは、退役または統合を検討する。観測手段の数を品質目標にしない。

## 独立検証

実装者がテストの成功を申告するだけでは、完了の証拠にならない。実用的な場合は、別のコンテキストまたは別のエージェントで次を監査する。

- 変更差分の全体とarchitectureの境界
- 製品コードの各変更が因果上必要であること
- counterfactualまたは注入した故障
- 無関係な変更とscope拡大
- 弱体化・削除されたテスト、または実装の詳細に依存したテスト

独立監査では、確認できた証拠と残る不確実性を報告する。対象を絞った検証は別途必要である。

## 停止条件

作業を終える前に、次の内容を証拠で確認する。全件テストの成功だけでは完了と判断しない。

- 要求された動作
- Architecture契約とdecision owner
- 対象を絞ったPrimary proof
- 関連する構造検査
- 説明不能な補償がないこと
- 依頼された範囲に収まっていること

依頼された範囲で作業を終える。不要な整理、隣接領域の監査、推測に基づくframeworkの実装へ先回りしない。

## 再利用可能なtask template

このテンプレートには、その作業で決めた内容を簡潔に記録する。欄を埋めることを完了目標にしない。適用できないフィールドだけを削除し、不明な詳細を想像で補ってはならない。

### 新しい概念・feature

```text
Goal:
Decision owners:
Root cause / hypothesis: user need and unresolved design assumptions
In scope:
Out of scope:
Forbidden compensations:
Expected production shape:
Primary proof:
Structural proof:
Counterfactual: invalid owner/path or feature-disabled behavior
Stop condition:
```

### バグ修正・regression

```text
Goal:
Decision owners:
Root cause / hypothesis:
In scope:
Out of scope:
Forbidden compensations: fallback, silent success, unrelated rebuild, test weakening
Expected production shape: minimal causal edit
Primary proof: fail before, pass after
Structural proof:
Counterfactual: re-injected semantic fault is detected
Stop condition:
```

### Architecture refactor

```text
Goal:
Decision owners:
Root cause / hypothesis: duplicated authority or unenforced boundary
In scope:
Out of scope:
Forbidden compensations: behavior change, compatibility layer, duplicated specification
Expected production shape: one owner and explicit consumers
Primary proof: behavior equivalence appropriate to the contract
Structural proof: invalid dependency or duplicate owner is rejected
Counterfactual: temporary boundary violation is detected
Stop condition:
```

architecture-significantな変更では、このテンプレートに前述のExpected ImpactとActual Impactを追加する。別の作業台帳へ複写しない。

## 新規projectの立ち上げ

実在する判断と責務を保護できる、最小のharnessから始める。

- 正本文書とcommandへの対応を示す短い`AGENTS.md`
- 状態の層、識別情報、ownership、dependency、トランザクションを定義する`docs/architecture.md`
- 契約中心の検証方針を定義する`docs/testing.md`
- ドメインが初めて現れる時点でのドメインの設計文書
- architecture manifestとgeneric arch lint engine
- 代表的なEnd-to-end path 1件
- 重要契約に対する、対象を絞ったPrimary proof
- formatter、lint、build、テストcommand
- 独立したレビューまたは監査の経路

立ち上げ時に、将来の設計全体を先回りして決めない。実在する判断と依存関係だけに境界を設ける。製品仕様はプロジェクトの設計文書へ記載し、共通harnessやmanifestへは複写しない。

## Goodhart化の防止

Harnessを採点表として扱わない。テストへのメタデータの一括付与、全契約のJSON/YAMLへの複写、テスト数や網羅率の閾値を品質目標にすること、source tokenだけから意味の正しさを主張すること、テンプレートの欄を埋めただけで成功とすることは避ける。重要契約に独立した故障検出の証拠があるか、不正なownershipを実用上最も強い境界で防いでいるかを評価する。
