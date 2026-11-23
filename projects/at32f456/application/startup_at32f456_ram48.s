; APP专用4KiB栈；向量及运行库入口复用原启动文件，Loader保持8KiB。
APP_STACK_BYTES EQU     0x00001000
                GET     startup_at32f456_app.s
                END
