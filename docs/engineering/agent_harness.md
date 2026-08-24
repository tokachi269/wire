# 移植可能なAgent Engineering Harness

このplaybookはagent支援engineeringのproject非依存workflowである。境界と検証の規律を定めるが、product semanticsの正本は各projectのarchitecture文書に置く。

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

1つのDecisionには1つのOwnerを置く。Consumerは同じinputから判断を繰り返さず、authoritative outputを使う。まだ存在しない概念のために抽象化を設計しない。

各契約の保護手段は次の順で選ぶ。

1. Invalid stateを構造的に表現不能にできるか。
2. Type、module、capabilityで防げるか。
3. 安定した境界をarchitecture lintで保護できるか。
4. Behavioral Primary proofが必要か。
5. 意味のあるmetamorphic relationがあるか。
6. Differential proofに使える等価経路があるか。
7. User-facing ScenarioまたはEnd-to-end proofが必要か。

異なるfault classには複数levelのevidenceが必要な場合がある。Token scanはbehaviorを証明せず、E2E scenarioはdependency ownershipを証明しない。Behavioral probeは`docs/testing.md`のsystem characteristic catalogから選び、quotaにしない。

## Guardrail昇格の段階

防御を無制限に積み上げない。基礎契約が明確になったら次の順に昇格させる。

```text
incident-specific assertion
  -> generalized behavioral contract
  -> architecture lint
  -> type / module / capability boundary
```

昇格後は元のfaultを再注入する。強い仕組みがそれを検出または防止でき、弱いguardに独立したoracleがない場合は、そのtestまたはguardを退役候補として評価する。

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

各production editは、root cause、このeditが必要な理由、このeditがなければfailするfocused proofで説明できなければならない。Fix後に新しいfailureが出た場合、補償behaviorを足す前に直前の変更を縮小またはrevertできるか調べる。実装を通すために既存契約を弱めない。

## 独立検証

実装者によるtest passの申告だけでは完了evidenceにならない。実用的なら別contextまたは別agentで次を監査する。

- 完全なdiffとarchitecture境界
- 各production editの因果上の必要性
- counterfactualまたは注入したfault
- 無関係な変更とscope拡大
- 弱体化・削除されたtest、または実装にcoupleしたtest

独立監査はevidenceと不確実性を報告するものであり、focused proofを代替しない。

## 停止条件

Full suiteがgreenであるだけでは不十分である。次をevidenceで証明できたときだけ停止する。

- 要求されたbehavior
- Architecture contractとDecision owner
- Focused Primary proof
- 関連するstructural check
- 説明不能な補償がないこと
- Scope containment

要求された境界で停止し、cleanup、隣接audit、推測によるframeworkへ先回りしない。

## 再利用可能なtask template

完了checklistの点取りではなく、小さなdecision recordとして使う。実際に適用不能なfieldだけを削除し、想像したdetailで埋めない。

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

## 新規projectの立ち上げ

実在するDecisionを保護できる最小のharnessから始める。

- 正本文書とcommandへの対応を示す短い`AGENTS.md`
- state layer、identity、ownership、dependency、transactionを定義する`docs/architecture.md`
- 契約中心の検証方針を定義する`docs/testing.md`
- domainが初めて現れる時点でのdomain architecture文書
- architecture manifestとgeneric architecture lint engine
- 代表的なEnd-to-end path 1件
- 重要契約のfocused Primary proof
- formatter、lint、build、test command
- 独立reviewまたはaudit path

立ち上げ時に将来のarchitecture全体を設計しない。実在するDecisionとdependencyにだけ境界を追加する。Project semanticsはportable harnessやmanifestではなくproject architectureに置く。

## Goodhart化の防止

Harnessはscoreではない。Testへのmetadata一括付与、全契約のJSON/YAMLへの複写、test数やcoverage thresholdの品質目標化、source tokenからのsemantics主張、template fieldを埋めたことによる成功判定を行わない。重要契約が独立したfault-detecting evidenceを持つか、invalid ownershipを実用上最も強い境界で防いでいるかを評価する。
