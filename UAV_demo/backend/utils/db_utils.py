# backend/utils/db_utils.py
import mysql.connector
from simulation_config import DB_CONFIG
import logging  # 用于记录数据库操作的日志
from datetime import datetime

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def get_db_connection():
    """建立并返回数据库连接"""
    try:
        conn = mysql.connector.connect(**DB_CONFIG)
        return conn
    except mysql.connector.Error as err:
        logger.error(f"数据库连接错误: {err}")
        return None


def create_table_if_not_exists():
    """检查并创建 uav_state_log 表 (如果它不存在)"""
    # (SQL建表语句已在外部执行，这里可以作为可选的程序内检查)
    # 为简化，假设表已由用户创建。
    # 若要程序内创建，可以在此执行上面的 CREATE TABLE SQL。
    pass


def insert_uav_state_log(simulation_time, uav_id, x, y, z, vx=0.0, vy=0.0, vz=0.0):
    """向数据库中插入一条无人机状态日志"""
    conn = get_db_connection()
    if not conn:
        return False

    cursor = conn.cursor()
    sql = """
        INSERT INTO uav_state_log (simulation_time, uav_id, x_pos, y_pos, z_pos, vx, vy, vz)
        VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
    """
    val = (simulation_time, uav_id, x, y, z, vx, vy, vz)

    try:
        cursor.execute(sql, val)
        conn.commit()
        # logger.info(f"记录已插入: UAV ID {uav_id} at time {simulation_time}")
        return True
    except mysql.connector.Error as err:
        logger.error(f"插入记录错误: {err}")
        conn.rollback()
        return False
    finally:
        if conn.is_connected():
            cursor.close()
            conn.close()


def get_uav_state_logs(limit=100, uav_id_filter=None):
    """从数据库获取最新的N条无人机状态日志，可按UAV ID过滤"""
    conn = get_db_connection()
    if not conn:
        return []

    cursor = conn.cursor(dictionary=True)  # 返回结果为字典形式

    query_parts = [
        "SELECT log_id, simulation_time, uav_id, x_pos, y_pos, z_pos, vx, vy, vz, logged_at FROM uav_state_log"]
    params = []

    if uav_id_filter is not None:
        query_parts.append("WHERE uav_id = %s")
        params.append(uav_id_filter)

    query_parts.append("ORDER BY logged_at DESC, simulation_time DESC LIMIT %s")  # 获取最新的记录
    params.append(limit)

    sql = " ".join(query_parts)

    try:
        cursor.execute(sql, tuple(params))
        results = cursor.fetchall()
        # 将 datetime 对象转换为字符串，以便JSON序列化
        for row in results:
            if isinstance(row, dict) and 'logged_at' in row and row['logged_at'] is not None:
                # 尝试转换为字符串，如果失败则使用默认值
                try:
                    row['logged_at'] = str(row['logged_at'])
                except Exception:
                    row['logged_at'] = None
        return results
    except mysql.connector.Error as err:
        logger.error(f"查询记录错误: {err}")
        return []
    finally:
        if conn.is_connected():
            cursor.close()
            conn.close()


def clear_uav_state_logs():
    """(可选) 清空日志表，用于重置仿真时"""
    conn = get_db_connection()
    if not conn:
        return False
    cursor = conn.cursor()
    sql = "DELETE FROM uav_state_log"  # 或者 TRUNCATE TABLE uav_state_log
    try:
        cursor.execute(sql)
        conn.commit()
        logger.info("uav_state_log 表已清空。")
        return True
    except mysql.connector.Error as err:
        logger.error(f"清空表错误: {err}")
        conn.rollback()
        return False
    finally:
        if conn.is_connected():
            cursor.close()
            conn.close()

# 在应用启动时可以调用一次建表函数 (如果选择在程序内建表)
# create_table_if_not_exists()