"""
MCP Server 最小示例（stdio 版）
================================

【运行流程】
1. 本脚本不单独交互运行 —— stdio 模式下启动后会静默等待协议消息（看起来像"卡住"），
   它是给客户端当子进程拉起的，不是给人用的。
2. 验证方式：运行客户端  python3 13_mcp_client.py
   流程：客户端以子进程启动本脚本 → 通过 stdin/stdout 管道收发 JSON-RPC
        → 发现工具列表（add） → 调用 add(10, 20) → 返回 30
3. 可选：以网络服务方式运行（会出现真实网址）
   ~/.local/bin/fastmcp run 12_mcp_server_fastmcp.py --transport http
   → http://127.0.0.1:8000/mcp
   注意：http 模式下 13/14 号 stdio 客户端无法连接。
"""
import sys
from fastmcp import FastMCP

# ==================== 1. 创建 MCP 实例 ====================
# "Math Server" 是服务名称，客户端连接时可见
mcp = FastMCP("Math Server")


def log(*args):
    """日志统一输出到 stderr（stdio 协议下 stdout 只能传 JSON-RPC 消息）"""
    print(*args, file=sys.stderr)


# ==================== 2. 提供 add 工具 ====================
# @mcp.tool() 把函数注册为 MCP 工具
# 类型提示 + 文档字符串会自动转换成工具 Schema，供大模型理解何时调用、如何传参
@mcp.tool()
def add(a: int, b: int) -> int:
    """
    计算两个整数的和 (Add two integers)

    Args:
        a: 第一个加数
        b: 第二个加数

    Returns:
        两数之和
    """
    result = a + b
    log(f"Executing add({a}, {b}) -> {result}")
    return result


# ==================== 3. 运行 Server ====================
if __name__ == "__main__":
    log("MCP Server is running...")
    mcp.run()  # 默认 stdio 传输：等待客户端拉起并通信
