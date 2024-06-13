# neuron 西门子s7 协议插件

## 说明:

1. 参考neuron modbustcp插件代码和snap7代码写的西门子s7协议插件

   [Modbus TCP | Neuron 文档 (neugates.io)](https://neugates.io/docs/zh/latest/configuration/south-devices/modbus-tcp/modbus-tcp.html "Modbus TCP | Neuron 文档 (neugates.io)")

   [Snap7 Homepage (sourceforge.net)](https://snap7.sourceforge.net/ "Snap7 Homepage (sourceforge.net)")
2. 基于neuron 2.6版本(测试2.6.5)
3. 基于西门子s7-1200/1500,其他s200可能不支持
4. 该插件属于个人之前自用,因为项目原因暂时也不会进行开发了.测试不多,发布属于交流,有愿意改bug加功能和完善建议热烈欢迎.
5. 功能和稳定性方面和官方的s7插件差距比较大,商用建议用官方的.

## 功能限制:

1. 支持网页提交单tag写入,不支持多tag写入;
2. 支持mutilread读取多tags,tag\_sort会进行组合排序;
3. 不支持非db块读写.

## 使用方法:

1. 文件放到neuron\plugins\s7下面;
2. 修改neuron的cmakelist参考其他插件位置添加
   ```c++
   add_subdirectory(plugins/s7)
   ```
3. 按官方编译出[libplugin-s7-tcp.so](http://libplugin-s7-tcp.so "libplugin-s7-tcp.so"),放到plugins目录下

   [源码构建 | Neuron 文档 (neugates.io)](https://neugates.io/docs/zh/latest/installation/compile.html "源码构建 | Neuron 文档 (neugates.io)")
4. 修改config的default\_plugins.json
   ```c++
   {
     "plugins": [
       "libplugin-mqtt.so",
       "libplugin-ekuiper.so",
       "libplugin-monitor.so",
       "libplugin-modbus-tcp.so",
       "libplugin-s7-tcp.so",
       "libplugin-modbus-rtu.so"
     ]
   }
   ```
