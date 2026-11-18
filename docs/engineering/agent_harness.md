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
3. 安定した境界をarchitecture lintで保護できるか。
4. Behavioral Primary proofが必要か。
5. 意味のあるmetamorphic relationがあるか。
6. Differential proofに使える等価経路があるか。
7. User-facing ScenarioまたはEnd-to-end proofが必要か。

異なる故障classには複数levelのevidenceが必要な場合がある。Token scanはbehaviorを証明せず、E2Eシナリオはdependency ownershipを証明しない。Behavioral probeは`docs/testing.md`のsystem characteristic catalogから選び、割当目標にしない。

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

製品コードの各変更について、根本原因、変更が必要な理由、変更しない場合に失敗する対象を絞った検証を説明する。Fix後に新しい失敗が出た場合、補償behaviorを足す前に直前の変更を縮小またはrevertできるか調べる。実装を通すために既存契約を弱めない。

## Architecture-significant change workflow

authoritative/persisted状態、public/Core API、操作意味論、Decision担当箇所、module/layer dependency、cross-layer/cross-domain dependency、fallback/special path、persistenceの変更はarchitecture-significantとして扱う。

expected scopeより実変更範囲が広がった場合も途中からこのworkflowへ切り替える。全ての小bugや局所変更へ機械的に適用しない。

architecture問題、繰り返すincident、新しいguardrailを検討する問題、既存分野が不明な問題では、実装前にSWEBOK v4を主taxonomyとしてproblemを既存software engineering knowledgeへ分類する。これはagent workflow内の調査であり、分類器、プロジェクト固有taxonomy、恒久ledgerを作らない。

複数Knowledge Areaを選んでよく、Systems Engineeringまで広がる場合だけSEBoKを補助にする。

```text
Problem:
Observed symptom:
Likely SWEBOK KA:
Established terminology:
Known techniques / literature:
Project-specific gap:
Need for a new Wire-specific mechanism:
```

既存技法がgapを満たす場合は名前を変えたframeworkを作らない。Wire固有mechanismが必要という欄は、既存技法・既存harness・既存authorityで埋まらない差だけを記述する。

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

ExpectedとActualが大きく異なる場合は補償実装を続けず、原因とscopeを評価する。特に第二のauthoritative representation、authoritative/derived意味の二重化、persisted mirror/cache、generic layerへのfeature固有概念の流入、guardを一時解除して既存操作を別経路から呼ぶこと、操作固有behaviorの下流materialization/viewerへの漏出、新しいfallback inference、expected impact外への説明不能な伝播は強い停止signalである。

Tokenやpathはreview candidateを示せるだけで、これらの意味を単独では証明しない。

完了時は三方向を分けてreviewする。

- Backward: 既存製品の仕様、architecture契約、テスト契約を壊していないか。
- Inward: 今回architecture上で実際に何を変えたか。ExpectedとActualの差を説明できるか。
- Forward: 次の変更を不必要に難しくするauthority、dependency、special pathを増やしていないか。完全な自動判定は主張しない。

Architecture observationのDSM、Reflexion Model、Architecture Delta、co-change、hotspotはreview evidenceであり、数値自体を失敗thresholdにしない。自動失敗は既存architecture lintなどが客観的に判定できる契約violationに限る。

強いmechanismへ吸収された、他sensorと同じ情報しか出さない、有用なsignalがない、maintenance costがreview価値を上回るsensorは退役または統合を評価する。sensor数を品質目標にしない。

## 独立検証

実装者によるテストpassの申告だけでは完了evidenceにならない。実用的なら別contextまたは別agentで次を監査する。

- 完全なdiffとarchitecture境界
- 各production editの因果上の必要性
- counterfactualまたは注入した故障
- 無関係な変更とscope拡大
- 弱体化・削除されたテスト、または実装にcoupleしたテスト

独立監査はevidenceと不確実性を報告するものであり、focused proofを代替しない。

## 停止条件

作業を終える前に、次の内容を証拠で確認する。全件テストの成功だけでは完了と判断しない。

- 要求されたbehavior
- Architecture契約とDecision担当箇所
- Focused Primary proof
- 関連するstructural check
- 説明不能な補償がないこと
- Scope containment

要求された境界で停止し、cleanup、隣接audit、推測によるframeworkへ先回りしない。

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

architecture-significantな場合は、このテンプレートに前述のExpected ImpactとActual Impactを追加する。別のtask ledgerへ複写しない。

## 新規projectの立ち上げ

実在するDecisionを保護できる最小のharnessから始める。

- 正本文書とcommandへの対応を示す短い`AGENTS.md`
- 状態layer、識別情報、ownership、dependency、トランザクションを定義する`docs/architecture.md`
- 契約中心の検証方針を定義する`docs/testing.md`
- ドメインが初めて現れる時点でのドメインarchitecture文書
- architecture manifestとgeneric architecture lint engine
- 代表的なEnd-to-end path 1件
- 重要契約のfocused Primary proof
- formatter、lint、build、テストcommand
- 独立reviewまたはaudit path

立ち上げ時に将来のarchitecture全体を設計しない。実在するDecisionとdependencyにだけ境界を追加する。製品仕様はプロジェクトの設計文書へ記載する。共通harnessやmanifestへは複写しない。

## Goodhart化の防止

Harnessはscoreではない。Testへのメタデータ一括付与、全契約のJSON/YAMLへの複写、テスト数やcoverage thresholdの品質目標化、source tokenからの意味論主張、テンプレートフィールドを埋めたことによる成功判定を行わない。重要契約が独立したfault-detecting evidenceを持つか、invalid ownershipを実用上最も強い境界で防いでいるかを評価する。
