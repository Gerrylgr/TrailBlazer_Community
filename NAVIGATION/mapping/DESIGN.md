## ordinary occupancy mapping pipeline:
```
启动后默认 NO_MAP，并去默认位置尝试加载地图（LOADING）：
    加载成功：MAP_READY
    加载失败（地图损坏）：ERROR
    未找到地图文件：NO_MAP

对于所有的 ERROR 状态:
    都需要先清除（CLEARING）才能重新建图，也就是 ERROR -> CLEARING -> NO_MAP

对于 BUILDING_NEW_MAP:
    必须在 NO_MAP/MAP_READY 状态的前提下

对于 BUILDING_INCREMENTAL:
    必须在 MAP_READY 状态的前提下

对于 SAVING:
    必须在 BUILDING_NEW_MAP/BUILDING_INCREMENTAL 的状态前提下；
    SAVING 成功后直接进入 MAP_READY 状态发布地图

则正常使用的流程可以为：
    第一次使用：
        LOADING -> NO_MAP -> BUILDING_NEW_MAP -> SAVING -> MAP_READY
    后边直接加载现成的地图：
        LOADING -> MAP_READY
    后边增量建图：
        LOADING -> MAP_READY -> BUILDING_INCREMENTAL -> SAVING -> MAP_READY
    后边加载地图出错时：
        LOADING -> ERROR -> CLEARING -> NO_MAP -> （重新建图）
```

groundCallback 和 nonGroundCallback 只在 BUILDING_NEW_MAP/BUILDING_INCREMENTAL 时运行（对应的 subscription 只有在服务被调用时实例化）<br><br>

对于 final_cells 和 occupancy map 之间的转换：<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;BUILDING_NEW_MAP/BUILDING_INCREMENTAL 时：处理一帧、发布一次地图<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;MAP_READY 时：使用定时器按照 map_publish_frequency_ 的频率发布静态地图<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;其他时候：不发布地图

### 注意
timer/service callbacks 中需要使用 node 时，**不能像下边这样将 node 作为参数通过 lambda 传入**：
```
state_timer_ =
    node->create_wall_timer(
        50ms,
        [this, node]()
        {
            this->stateTimerCallback(node);
        });
```
这里的 lambda 会将 node 永久保存，从而再次形成循环引用：
```
Server
  │
  │ shared
  ▼
Plugin
  │
  │ shared
  ▼
Timer
  │
  │ owns callback
  ▼
Lambda
  │
  │ shared node
  ▼
Server
```

解决办法是直接在需要时 auto node = node_.lock(); 获取指针临时使用即可