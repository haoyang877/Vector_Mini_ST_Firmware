# 协议说明合并 v1

按用户要求，只保留 `docs/protocols/README.md` 这一份现行通信协议说明。
依据当前 `protocol.c/.h`、AppProtocol、公共帧编解码及 Loader 核对字段和支持范围；
删除旧版、重复说明和已退役提案，不保留跳转占位页。

参数 Flash 存储属于参数模块，其仍有效的规则合并回该模块 README。
JSON 是回归数据而非说明书，移入 `tests/fixtures/protocol/`，内容保持逐字节不变。
更新代码中的夹具路径及所有文档引用，保留历史计划和测试报告的原始结果。

本轮不修改固件行为。开始时文档、脚本及哈希副本在
`outputs/protocol-docs-cleanup/baseline/`；回退仅恢复本轮文件，不覆盖其他工作区修改。
验收为唯一现行文档、完整有效链接、夹具哈希不变和 PR 检查通过。

## 完成

已删除 68 份重复/过时 Markdown（含旧协议目录和四份旧通信架构提案），
现行协议仅保留 docs/protocols/README.md，共 238 行。
按代码纠正能力位图、六种运行模式、目标重复处理、参数不支持、标定和升级字段。
参数存储规则合并到 firmware/parameters/README.md，不作为在线通信能力。

29 个 JSON 夹具移入 tests/fixtures/protocol/，逐文件 SHA-256 与移动前完全一致。
8 个实际消费者同步路径；历史向量生成器不再读取或重建已删除的旧 Markdown。
更新文档索引和全部本地链接，保留历史计划/报告的测试结论。

完整 PR 检查 PASS：outputs/runs/protocol-docs-one-20261103/summary.json。
范围清单、前态和哈希记录位于 outputs/protocol-docs-cleanup/。
没有修改固件行为、没有烧录。用户随后询问的是 protocol.c 的职责拆分建议，尚未执行新的源码拆分。
