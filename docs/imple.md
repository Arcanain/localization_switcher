# 1029

ESG?

グラフが提供する機能は何か．
* [ ] 構造体を受け取って，その構造体に含まれる情報からノードを一つ返す
* [ ] 構造体と共に受け渡す関数の設計→これはgraphに引数？か何かで渡す形式が良さそう
* [ ] 

* [ ] 構造体の用意が必要→lifecycle clientでget state→それをまとめる
* [ ] 構造体を作ってから渡すのか，渡してからcomponentで作るのか
* [ ] 


- componentが持つ変数
  - 
- componentが持つメソッド
  - 

# Localization Switcher Node 設計仕様書

**版:** v0.9（設計凍結／実装前）
**日付:** 2025-10-11
**対象:** ROS2（rclcpp / Lifecycle）
**著者:** （あなたのプロジェクト名）

---

## 1. 背景 / コンテキスト

自律移動ロボットでは、GNSS、EMCL（拡張モンテカルロ局所化）、将来的なLIDAR/VIO など**複数の自己位置推定（Localization）手法**を状況に応じて切り替える必要があります。
本仕様では、**切り替え判定と実行を担う「Localization Switcher」ノード**を設計し、**責務の分離**と**拡張性（戦略差し替え）**を最大化します。

---

## 2. スコープ（Scope）

本ノードが担う範囲：

* **ROS I/O 層:**

  * /odom, GNSS fix 等の購読（観測の取得）
  * 各Localizationノード（GNSS/EMCL）のLifecycle状態の取得・切り替え（configure/activate/deactivate/cleanup）
  * 周期タイマでの評価・実行

* **意思決定オーケストレーション:**

  * 切り替え戦略（Strategy）と状態遷移グラフ（Graph）を利用し、**どこに遷移すべきか**を決定・実行

スコープ外（Not in scope）：

* 各Localizationアルゴリズム自身の実装やパラメータ調整
* 経路計画・障害物回避（本ノードは関与しない）
* センサー前処理（位置推定の元データ生成）

---

## 3. 目的（Goals）

* **安全:** 許可された遷移のみ実行し、禁止状態（例：両方active）を回避
* **拡張性:** 切り替え判定ロジック（Strategy）を**実行中でも差し替え可能**な設計
* **独立性:** ROS依存はNode層に限定。**Component/Strategy/Graphは純C++**
* **テスト容易性:** Graph/Strategyを**単体テスト**可能に
* **運用性:** YAMLでGraph・Strategyを設定、ログ/診断で可視化

---

## 4. 用語 / 主要コンセプト

* **Node:** ROS I/O とLifecycle操作を行う層（実行担当）
* **Component:** Graph/Strategy を保持し、**意思決定をオーケストレーション**する層（ROS非依存）
* **Strategy:** 切り替え判定ロジックを実装（**時間依存は内部保持**）。Factoryで生成・差し替え
* **Graph:** 許容される状態（ノード）と遷移（エッジ＋レシピ）を表現する**静的構造**
* **Recipe:** 遷移実行手順（ActionStepの配列）

---

## 5. アーキテクチャ全体像

```
ROS Node (LocalizationSwitchNode)
 ├─ Subscribe: /odom, /fix（等）
 ├─ Lifecycle clients: GNSS, EMCL
 ├─ Timer → onTimer()
 │    └─ component.tick(now, world, managed)
 │          ├─ graph.resolve(managed) → 現在モード
 │          ├─ strategy.desiredTarget(current, world, managed, now) → 目標モード
 │          ├─ graph.shortestAllowedPath(current, target) → 先頭エッジ.recipe
 │          └─ Decision{from,to,recipe,reason}
 ├─ execute(recipe) → Lifecycle操作
 └─ component.notifyTransitionResult(result, now) → strategy.onTransition()
```

**依存方向:** `Node → Component → (Strategy, Graph)`（逆依存なし）

---

## 6. データ契約（共通型：ROS非依存）

> `include/localization_switcher/component/common_types.hpp`

* **WorldState（POD）**

  * `double x, y;`（地図座標）
  * `bool fix;`（GNSS fix 有無）
  * *拡張余地:* `double fix_quality, velocity` など
  * **設定:** Node が購読値から詰める／**参照:** Component/Strategy

* **ManagedState（POD）**

  * `bool gnss_active;`
  * `bool emcl_active;`
  * **設定:** Node がLifecycleから詰める／**参照:** Component/Graph

* **ActionStep**

  * `std::string target_node_name;`（"gnss" | "emcl"）
  * `std::string operation;`（"configure" | "activate" | "deactivate" | "cleanup"）
  * `double timeout_s;` / `int retry`(任意)

* **TransitionRecipe**

  * `std::vector<ActionStep> steps;`
  * `std::string description`(任意)

* **Decision**（Component→Node）

  * `std::string from, to;`（ModeNodeId）
  * `TransitionRecipe recipe;`
  * `std::string reason;`（説明/デバッグ）

* **ExecResult**（Node→Component）

  * `bool success;`
  * `std::string message;`

---

## 7. 状態遷移グラフ（Graph）

> `include/localization_switcher/component/graph.hpp`

### 7.1 モード定義（例）

* `GNSS_ONLY  = {gnss_active:true,  emcl_active:false}`（allowed:true）
* `EMCL_ONLY  = {gnss_active:false, emcl_active:true }`（allowed:true）
* `BOTH_ACTIVE   = {true,true }`（**allowed:false** / 到達禁止）
* `BOTH_INACTIVE = {false,false}`（allowed:true / 安全停止用など運用で選択）

### 7.2 エッジ（遷移）

* `from` → `to` に **TransitionRecipe** を紐付け
* 例：`EMCL_ONLY → GNSS_ONLY`

  * steps: `configure(gnss) → activate(gnss) → deactivate(emcl)`

### 7.3 Graph API（宣言レベル）

```cpp
class ModeGraph {
public:
  bool loadFromYaml(const std::string& path);
  bool validate() const;

  std::string resolve(const ManagedState& managed) const;  // 現在モードID
  bool allowed(const std::string& node_id) const;

  std::vector<const ModeEdge*> shortestAllowedPath(
      const std::string& from, const std::string& to) const;

  const ModeEdge* edge(const std::string& from, const std::string& to) const;
};
```

### 7.4 YAML スキーマ（`modes_graph.yaml`）

```yaml
nodes:
  - id: GNSS_ONLY
    semantic: { gnss_active: true,  emcl_active: false }
    allowed: true
  - id: EMCL_ONLY
    semantic: { gnss_active: false, emcl_active: true }
    allowed: true
  - id: BOTH_ACTIVE
    semantic: { gnss_active: true,  emcl_active: true }
    allowed: false
  - id: BOTH_INACTIVE
    semantic: { gnss_active: false, emcl_active: false }
    allowed: true

edges:
  - from: GNSS_ONLY
    to: EMCL_ONLY
    recipe:
      steps:
        - { target_node_name: "emcl", operation: "configure",  timeout_s: 5.0 }
        - { target_node_name: "emcl", operation: "activate",   timeout_s: 5.0 }
        - { target_node_name: "gnss", operation: "deactivate", timeout_s: 5.0 }

  - from: EMCL_ONLY
    to: GNSS_ONLY
    recipe:
      steps:
        - { target_node_name: "gnss", operation: "configure",  timeout_s: 5.0 }
        - { target_node_name: "gnss", operation: "activate",   timeout_s: 5.0 }
        - { target_node_name: "emcl", operation: "deactivate", timeout_s: 5.0 }
```

**validate()** は、未定義ノード参照、禁止ノードへの到達、孤立ノード等をチェック。

---

## 8. Strategy（判定ロジック）層

> `include/localization_switcher/component/strategy.hpp`（抽象）
> `include/localization_switcher/component/strategy_waypoint_sequence.hpp`（具象）

### 8.1 共通IF（抽象）

```cpp
class ISwitchStrategy {
public:
  virtual ~ISwitchStrategy() = default;

  virtual bool configure(const std::string& yaml_path) = 0;

  virtual std::optional<std::string> desiredTarget(
      const std::string& current_node_id,
      const WorldState& world,
      const ManagedState& managed,
      const std::chrono::steady_clock::time_point& current_time) = 0;

  virtual void onTransition(
      const ExecResult& result,
      const std::chrono::steady_clock::time_point& current_time) = 0;
};
```

**時間依存（ヒステリシス／クールダウン等）は、すべて Strategy 内部で `std::chrono` により管理。**
Node/Component は時間計算を行わない。

### 8.2 WaypointSequenceStrategy（先行実装）

* 領域（円）に入って一定時間滞在 → その領域の `prefer_node` へ切替
* **内部状態例:**

  * `last_switch_time_`, `cooldown_until_`
  * `entry_time_`, `current_region_name_`
* **タイミング:** `entry_hold_s`, `exit_hold_s`, `cooldown_s`

**YAML（`strategy_waypoint.yaml`）**

```yaml
type: waypoint_sequence
timing:
  entry_hold_s: 2.0
  exit_hold_s: 1.0
  cooldown_s: 5.0
regions:
  - name: area_A
    x: 10.0
    y: 5.0
    radius: 3.0
    prefer_node: GNSS_ONLY
    priority: 1
  - name: area_B
    x: 25.0
    y: 15.0
    radius: 5.0
    prefer_node: EMCL_ONLY
    priority: 2
```

---

## 9. StrategyFactory（拡張ポイント）

> `include/localization_switcher/component/strategy_factory.hpp`

* 目的：**Component を具体戦略から切り離す**（疎結合）
* `StrategyFactory::create(type)` で任意のStrategyを生成
* 走行中の**戦略差し替え**も可能（抽象ポインタ再代入）

```cpp
// 概念イメージ（実装ではなく設計）
std::shared_ptr<ISwitchStrategy> StrategyFactory::create(const std::string& type);
// 登録関数 registerDefaultStrategies() を一度呼ぶ
```

---

## 10. Component（意思決定オーケストレータ）

> `include/localization_switcher/component/localization_switcher_component.hpp`

* **tick(now, world, managed) → Decision**

  1. `current = graph.resolve(managed)`
  2. `target = strategy.desiredTarget(current, world, managed, now)`
  3. `path = graph.shortestAllowedPath(current, target)`
  4. 先頭エッジの `recipe` を含む `Decision` を返す（切替不要なら no-op）

* **notifyTransitionResult(result, now)**

  * `strategy.onTransition(result, now)` を呼び、内部タイマを更新

* **initialize(graph_yaml, strategy_yaml[, type])**

  * Graph 読込 → validate
  * StrategyFactory 経由で生成 → configure
  * 失敗時は上位（Node）に初期化失敗を返す

---

## 11. Node（ROS インターフェース）宣言

> `include/localization_switcher/node/localization_switcher_node.hpp`

```cpp
class LocalizationSwitchNode : public rclcpp::Node
{
public:
  LocalizationSwitchNode();
  ~LocalizationSwitchNode() = default;

private:
  // メイン周期
  void onTimer();

  // 購読
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  // Lifecycle
  void pollManagedStates();
  void executeTransitionRecipe(const TransitionRecipe &recipe);
  void notifyTransitionResult(const ExecResult &result,
                              const std::chrono::steady_clock::time_point &now);

  // Build states
  WorldState buildWorldState() const;
  ManagedState buildManagedState() const;

  // メンバ
  std::shared_ptr<LocalizationSwitcherComponent> component_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr gnss_lc_client_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr emcl_lc_client_;
  WorldState world_;
  ManagedState managed_;
  bool executing_transition_{false};
  std::chrono::steady_clock::time_point last_tick_time_;
};
```

---

## 12. 初期化フロー / YAML ロード

**Node 起動 → Component.initialize()**

```
Node:
  - パラメータ取得:
      graph_yaml_path
      strategy_yaml_path
      (strategy_type: 任意フォールバック)
  - Component生成 → initialize(graph_yaml, strategy_yaml, strategy_type)
Component.initialize():
  1) graph.loadFromYaml() → graph.validate()
  2) registerDefaultStrategies()（一度だけ）
  3) type = strategy_yaml内の"type"（なければ strategy_type / 既定値）
  4) strategy = StrategyFactory::create(type)
  5) strategy.configure(strategy_yaml)
```

**初回tick前:**
`graph.resolve(managed)` で現在モード判定。未定義/不定なら安全側へ修復Decisionを返せる設計も可。

---

## 13. 実行時フロー（タイマ駆動）

```
onTimer():
  now    = steady_clock::now()
  world  = buildWorldState()
  managed= buildManagedState()

  decision = component.tick(now, world, managed)

  if decision.action == "transition":
     executeTransitionRecipe(decision.recipe)
     result = ExecResult{success, message}
     notifyTransitionResult(result, now)
```

**実行原則（Recipe）**

* steps を順次（同期）実行
* タイムアウト/再試行は ActionStep の指定に従う
* 途中失敗で停止し、`success=false` を通知
* 競合防止：`executing_transition_` で多重実行を抑止

---

## 14. ROS 外部インターフェース

**パラメータ（Node）**

* `localization_switcher.graph_yaml_path: string`
* `localization_switcher.strategy_yaml_path: string`
* `localization_switcher.strategy_type: string`（任意）
* `localization_switcher.timer_period_ms: int`（周期）
* `localization_switcher.safe_mode: string`（例："EMCL_ONLY"）

**購読**

* `/odom`（`nav_msgs/msg/Odometry`）→ world_.x, world_.y
* `/fix`（`sensor_msgs/msg/NavSatFix`）→ world_.fix（簡約）

**サービス（Lifecycle）**

* `/<node_name>/change_state`（`lifecycle_msgs/srv/ChangeState`）× GNSS/EMCL

---

## 15. エラー処理 / フェイルセーフ

| 局面             | 失敗例              | ふるまい                              |
| -------------- | ---------------- | --------------------------------- |
| Graph 読込/検証    | YAML不整合          | 初期化失敗を上位に返す（ノード停止または再試行）          |
| Strategy 生成/設定 | type未知、必須キー欠落    | 初期化失敗                             |
| 経路探索           | target未到達        | Decisionなし or 修復Decision          |
| Recipe実行       | LCサービス失敗/timeout | `onTransition(fail)` 通知、必要に応じ安全側へ |

**禁止状態（BOTH_ACTIVE）** は Graph 側で `allowed:false`。
実機がこの状態に在ると判定したら**修復Decision**を返す実装を許容。

---

## 16. 非機能要件

* **リアルタイム性:** onTimer周期に対し、`tick()` と実行判定が十分に短いこと
* **スレッド安全:** `tick()` の呼び出しは単一スレッドで行い、Recipe実行中は新規実行を抑止
* **観測性:** Decisionログ、Recipe実行ログ、現在モードを出力（/diagnostics等は任意）
* **拡張性:** StrategyFactory で戦略追加／差し替え／実行中切替が可能
* **テスト容易性:** Graph/Strategy はROSなしでgtest可能

---

## 17. 代替案と採用理由（設計判断の記録）

* **時間管理:**

  * （採用）**Strategy 内部で `std::chrono` による時間管理**（A案）
  * （不採用）Node/Component前計算（B案）
    → 判定ロジックの一貫性と戦略ごとの自由度を優先

* **データ集約:**

  * （採用）**WorldState/ManagedState をPODで保持**
  * （不採用）分解引数
    → 拡張性・可読性・テスト性のため

* **Snapshot 構造:**

  * （不採用）冗長のため削除（`tick(now, world, managed)`で十分）

* **Strategy生成:**

  * （採用）**StrategyFactory**
  * （不採用）Componentが具体クラスを直接生成
    → 疎結合・ホットスワップ・設定切替性

---

## 18. テスト方針（概要）

* **Graph 単体:**

  * YAML読込、validate、resolve、path探索の正常系/異常系
* **Strategy 単体（WaypointSequence）:**

  * 位置・時間を疑似進行し、entry_hold/cooldown などの境界値テスト
* **Component 統合:**

  * モックGraph/Strategyで Decision 生成の整合
* **Node 統合（ROS）:**

  * ダミーLifecycleノードと結合、Recipeが順序通り実行されるか

---

## 19. 将来拡張（ロードマップ）

* **ContinuousFixStrategy**（fix継続時間ベース）
* **複合戦略（Hybrid）**
* **Graph/Strategy のホットリロード**（サービス経由）
* **pluginlib 化**（Strategyの動的ローディング）
* **診断トピック/サービス**の追加（現在モード、次の候補、内部タイマなど）

---

## 20. 付録：宣言スケルトン（実装しない）

> 参考：**宣言のみ**（中身はこの仕様に従って実装）

```cpp
// component/common_types.hpp
struct WorldState { double x{0}, y{0}; bool fix{false}; };
struct ManagedState { bool gnss_active{false}; bool emcl_active{false}; };

struct ActionStep { std::string target_node_name; std::string operation; double timeout_s{5.0}; int retry{0}; };
struct TransitionRecipe { std::vector<ActionStep> steps; std::string description; };

struct Decision { std::string from; std::string to; TransitionRecipe recipe; std::string reason; };
struct ExecResult { bool success{false}; std::string message; };
```

```cpp
// component/strategy.hpp
class ISwitchStrategy {
public:
  virtual ~ISwitchStrategy() = default;
  virtual bool configure(const std::string& yaml_path) = 0;
  virtual std::optional<std::string> desiredTarget(
      const std::string& current_node_id,
      const WorldState& world,
      const ManagedState& managed,
      const std::chrono::steady_clock::time_point& current_time) = 0;
  virtual void onTransition(const ExecResult& result,
                            const std::chrono::steady_clock::time_point& current_time) = 0;
};
```

```cpp
// component/graph.hpp（概念）
class ModeGraph {
public:
  bool loadFromYaml(const std::string& path);
  bool validate() const;
  std::string resolve(const ManagedState& managed) const;
  bool allowed(const std::string& node_id) const;
  std::vector<const ModeEdge*> shortestAllowedPath(const std::string& from, const std::string& to) const;
  const ModeEdge* edge(const std::string& from, const std::string& to) const;
};
```

```cpp
// component/strategy_factory.hpp（概念）
class StrategyFactory {
public:
  using Creator = std::function<std::shared_ptr<ISwitchStrategy>()>;
  static std::shared_ptr<ISwitchStrategy> create(const std::string& type);
  static void registerCreator(const std::string& type, Creator creator);
};
```

```cpp
// node/localization_switcher_node.hpp（概念）
class LocalizationSwitchNode : public rclcpp::Node {
  // onTimer, onOdom, onFix, pollManagedStates, executeTransitionRecipe,
  // notifyTransitionResult, buildWorldState, buildManagedState
};
```

---

以上が、**実装前に合意した正式な設計仕様書**です。
このドキュメントをベースに、順序としては **共通型 → Graph → Strategy → Component → Node** の順で実装に入ることを推奨します。
必要であれば、この仕様書をMarkdown/Confluence向けに再整形した版も用意します。



```
include/localization_switcher/
  component/
    common_types.hpp
    graph.hpp
    strategy.hpp
    strategy_waypoint_sequence.hpp
    strategy_factory.hpp
    localization_switcher_component.hpp        ← ★ 追加（宣言）
  node/
    localization_switcher_node.hpp

src/
  component/
    graph.cpp
    strategy_waypoint_sequence.cpp
    strategy_factory.cpp
    localization_switcher_component.cpp        ← ★ 追加（実装）
  node/
    localization_switcher_node.cpp

config/
  managed_nodes.yaml                           ← ★ 追加（キー定義 & LC判定規則）
  modes_graph.yaml
  strategy_waypoint.yaml

test/
  test_graph.cpp
  test_strategy_waypoint.cpp
  test_component.cpp                           ← 任意（Decision生成の統合テスト）
```