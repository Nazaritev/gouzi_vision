#!/usr/bin/env python3
import os
from typing import Optional

RESET = "\033[0m"

STYLES = {
    'error': "\033[1;31m",
    'warn': "\033[1;33m",
    'success': "\033[1;32m",
    'transition': "\033[1;36m",
    'state': "\033[1;35m",
    'detection': "\033[1;34m",
    'trigger': "\033[1;95m",
    'serial': "\033[0;96m",
    'debug': "\033[0;94m",
    'caution': "\033[0;33m",
}

SUCCESS_KEYWORDS = (
    '到点成功',
    '已到达 waypoint',
    '全部 waypoint 已完成',
    '触发横栏跳跃',
    'NavExecutor 已连接 Nav2',
)
TRANSITION_KEYWORDS = (
    '发送目标点',
    '发送直驱覆盖点',
    '切换到下一个 waypoint',
    '收到导航目标',
    'Local spin',
)
STATE_KEYWORDS = (
    '发布模式',
    '发布上坡步长滤波',
    '发布路径修正profile',
    'Updated serial mode',
    'Updated uphill step floor segment state',
    'Updated path correction profile',
    'Updated fixed step override runtime state',
)
DETECTION_KEYWORDS = (
    '最近斜坡:',
    '最近横栏:',
    '最近杆:',
    '最近限高杆:',
    '最近上台阶:',
)
CAUTION_KEYWORDS = (
    '未检测到有效',
    'orange_foreground_suppressed',
    'reject:',
    'no_candidate',
)
TRIGGER_KEYWORDS = ('触发状态更新',)
SERIAL_KEYWORDS = ('serial tx:', 'serial tx once:')
DEBUG_KEYWORDS = ('pose_controller:', 'step_debug', 'waypoint=')


def colorize_log(text: str, level: str = 'info', category: Optional[str] = None) -> str:
    if not text or os.getenv('NO_COLOR'):
        return text

    style = STYLES.get(_pick_category(text, level, category))
    if not style:
        return text
    return f'{style}{text}{RESET}'


def _pick_category(text: str, level: str, category: Optional[str]) -> Optional[str]:
    if category and category != 'auto':
        return category
    if level == 'error':
        return 'error'
    if level == 'warn':
        return 'warn'
    if _contains_any(text, SUCCESS_KEYWORDS):
        return 'success'
    if _contains_any(text, TRANSITION_KEYWORDS):
        return 'transition'
    if _contains_any(text, STATE_KEYWORDS):
        return 'state'
    if _contains_any(text, DETECTION_KEYWORDS):
        return 'detection'
    if _contains_any(text, TRIGGER_KEYWORDS):
        return 'trigger'
    if _contains_any(text, SERIAL_KEYWORDS):
        return 'serial'
    if _contains_any(text, DEBUG_KEYWORDS):
        return 'debug'
    if _contains_any(text, CAUTION_KEYWORDS):
        return 'caution'
    return None


def _contains_any(text: str, keywords: tuple[str, ...]) -> bool:
    return any(keyword in text for keyword in keywords)
