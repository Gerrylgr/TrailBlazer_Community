## TrailBlazer 感知模块

### 原 TrailBlazer 包组织架构：(以点云转换模块为例)
``` 
PointcloudCorrectorServer
    ↓ pluginlib 加载
PointCloudAligner 插件
    ↓
订阅点云/里程计，发布裁剪点云
``` 

### 如今 TrailBlazer_Community 感知模块的包组织架构：(添加了生命周期管理，但仍是一个功能模块一个 server->plugin)
``` 
PointcloudCorrectorServer LifecycleNode
    on_configure()
        ↓ 加载插件、声明参数、创建 publisher
    on_activate()
        ↓ 激活 publisher、创建 subscription
    on_deactivate()
        ↓ 关闭 subscription、停用 publisher
    on_cleanup()
        ↓ 释放插件、清空缓存
``` 

### 其余需要注意的：
 1. 插件内部节点使用 WeakPtr 防止循环引用：<br>
    #### 如果 server 节点和插件节点都使用强引用 shared_ptr：<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;节点拥有插件 (plugin_instance_)，引用计数 +1；<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;插件拥有节点 (shared_ptr<Node>)，引用计数 +1；<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;***此时在 server 节点销毁时，由于插件的 shared_ptr<Node> 还有一次引用计数，导致节点无法正常销毁⚠️***<br>
    #### 反之如果插件持有 weak_ptr<Node>：<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;节点拥有插件，引用计数 +1；<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;插件持有节点的 weak_ptr，引用计数不增加；<br>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;***此时当节点被销毁时，引用计数变为 0，节点正常释放。插件里的 weak_ptr 会自动失效（调用 .lock() 会返回空指针）***<br>
