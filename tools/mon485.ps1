# mon485.ps1 —— 485 通讯监控（打印机原始指令 + 本机回复 + WEB日志 + 电机动作事件）
#
# 由 mon485.bat 启动，也可直接：
#   powershell -NoProfile -ExecutionPolicy Bypass -File mon485.ps1
#   powershell ... -File mon485.ps1 -Ip 192.168.2.100 -Ms 50
#
# 颜色约定：
#   [RX 青色]  打印机 -> 本机
#   [TX 绿色]  本机 -> 打印机
#   [灰]       WEB 日志 / 状态行
#   [黄]       ★电机动作事件（电机方向 / 离合 / 动作 / 打印阶段 发生变化）
#   [红]       错误类日志、丢帧告警
#
# 输出同时写入 tools\logs\mon485_yyyyMMdd_HHmmss.log（纯文本，不截断），
# 控制台滚掉的都在文件里。
#
# 需要固件 v1.1.3+（/bus485raw 接口自那版起才有）。
#
# ★ 关于"前面的消息被覆盖"：
#   固件端 485 环形缓冲只有 32 帧，设备忙时每秒 60~90 帧 —— 只够存 400ms 左右。
#   所以脚本这边按"抓帧优先"设计：
#     1) 主循环几乎只干一件事：拉 /bus485raw，并用内层追平循环一次拉干净；
#     2) /log 和 /status 降频到 400ms 一次，不让它们占住抓帧的空档；
#     3) 每帧带 seq 号，一旦 seq 不连续立刻打红字告警 —— 有没有丢，一眼就知道。
#   固件一行不改。

param(
    [string]$Ip = "",
    [int]$Ms = 0
)

$ErrorActionPreference = 'Stop'

# ---------- 参数（命令行优先，其次环境变量，最后默认） ----------
$ip = $Ip
if (-not $ip) { $ip = $env:AMS_IP }
if (-not $ip) { $ip = "192.168.2.100" }

$iv = $Ms
if ($iv -le 0) {
    if ($env:AMS_MS) { $iv = [int]$env:AMS_MS } else { $iv = 50 }
}
if ($iv -lt 20) { $iv = 20 }      # 别把设备打爆

$base = "http://$ip"

# ---------- 日志文件 ----------
$root = $PSScriptRoot
if (-not $root) { $root = Split-Path -Parent $MyInvocation.MyCommand.Path }
$logDir = Join-Path $root "logs"
$logSw = $null
$logFile = ""
try {
    if (-not (Test-Path $logDir)) { New-Item -ItemType Directory -Force -Path $logDir | Out-Null }
    $logFile = Join-Path $logDir ("mon485_{0}.log" -f (Get-Date -Format "yyyyMMdd_HHmmss"))
    $logSw = New-Object System.IO.StreamWriter($logFile, $true, (New-Object System.Text.UTF8Encoding($false)))
    $logSw.AutoFlush = $true
} catch {
    Write-Host "[!] 日志文件创建失败（$($_.Exception.Message)），仅屏幕输出" -ForegroundColor Yellow
}

# ---------- 输出 ----------
# Write-Host 在 PS5.1 走宿主 UI，一秒钟上百行时会明显拖慢抓帧；
# 能用 Console 直写就用 Console（快 3~5 倍），不能就退回 Write-Host。
$fastConsole = $true
try { [Console]::ForegroundColor = [ConsoleColor]::Gray; [Console]::ResetColor() }
catch { $fastConsole = $false }

function Emit {
    param([string]$Text, [string]$Color = "Gray")
    if ($script:fastConsole) {
        try {
            [Console]::ForegroundColor = [ConsoleColor]$Color
            [Console]::WriteLine($Text)
            [Console]::ResetColor()
        } catch {
            $script:fastConsole = $false
            Write-Host $Text -ForegroundColor $Color
        }
    } else {
        Write-Host $Text -ForegroundColor $Color
    }
    if ($null -ne $script:logSw) {
        try { $script:logSw.WriteLine($Text) } catch { }
    }
}

function Get-J($path) {
    try { return Invoke-RestMethod -Uri ($script:base + $path) -TimeoutSec 3 -UseBasicParsing } catch { return $null }
}
function TS { Get-Date -Format "HH:mm:ss.fff" }

# ---------- 485 运动帧解码（3D C5 ... 短包） ----------
# 字节布局：3D C5 <len> <flag> <type> <ams> <statu> <motion> <?> <read>
# hex 位置：0-1  2-3   4-5   6-7    8-9   10-11 12-13   14-15   16-17 18-19
# 索引     ： 0    1     2     3      4      5     6       7       8     9
#
# ★ 只有 type=0x04（Dxx，「读/改运动状态」）会驱动固件状态机，
#   它的 ams/statu/motion/read 才在上述偏移上。
#   type=0x03（Cxx，「读运动信息」）只读不改状态，字段位置也不同（motion 在 b[8]），
#   两套偏移混着解会读出一堆假状态 —— 所以这里只解 Dxx，其余原样返回不猜。
function Decode-Frame([string]$hex) {
    if ($hex.Length -lt 20) { return "" }
    if ($hex.Substring(0, 2).ToUpper() -ne "3D") { return "" }
    # flag 的 bit7 决定长度字段位置：置位=短头（len 在 b2、crc8 在 b3），
    # 否则长头（len 在 b4、crc8 在 b6）。下面的偏移只对短头成立。
    $flag = 0
    try { $flag = [Convert]::ToInt32($hex.Substring(2, 2), 16) } catch { return "" }
    if (($flag -band 0x80) -eq 0) { return "" }        # 只解短头
    if ($hex.Substring(8, 2) -ne "04") { return "" }   # 只解 Dxx

    try {
        $ams    = [Convert]::ToInt32($hex.Substring(10, 2), 16)
        $statu  = [Convert]::ToInt32($hex.Substring(12, 2), 16)
        $motion = [Convert]::ToInt32($hex.Substring(14, 2), 16)
        $read   = [Convert]::ToInt32($hex.Substring(18, 2), 16)
    } catch { return "" }

    $t = ""
    switch ($statu) {
        0x03 { $t = "送料" }
        0x09 {
            if     ($motion -eq 0xA5) { $t = "准备使用" }
            elseif ($motion -eq 0x3F) { $t = "准备退料" }
            else                      { $t = "0x09" }
        }
        0x07 {
            if ($motion -eq 0x00) { $t = "暂停" } else { $t = "使用中(07)" }
        }
        0x0F {
            if ($motion -eq 0x00) { $t = "暂停(0F)" } else { $t = "使用中(0F)" }
        }
        0x01 { $t = "停止/空闲" }
        default { $t = "" }
    }
    if (-not $t) { return "" }
    return ("  <<{0} ams{1} read=0x{2:X2}" -f $t, $ams, $read)
}

# ---------- 共享状态 ----------
$lastSeq    = 0
$lastLogSeq = -1
$lastLogTxt = ""
$dupCount   = 0
$lastStatus = ""
$lastSum    = ""
$rawBad     = 0
$rawGone    = $false
$seqPrev    = -1      # 上一帧的 seq，用于丢帧检测
$lostTotal  = 0       # 累计丢帧数
$frameTotal = 0

# 电机动作事件用的"上一次快照"
$mPrev = @{
    dir   = $null   # 电机方向文本
    eng   = -1      # 离合掩码
    act   = ""      # 动作（need_send_out / need_pull_back / waiting）
    phase = ""      # 打印阶段（none / before_on_use / on_use / stop_on_use）
    pct   = -1      # 占空比
}

# ================= WEB 日志 =================
function Show-Log {
    $l = Get-J "/log"
    if ($null -eq $l) { return }

    if ($l.log -is [string]) {
        if ($l.log -and $l.log -ne $script:lastLogTxt) {
            Emit ("[{0}] LOG(旧格式) {1}" -f (TS), $l.log) "Gray"
            $script:lastLogTxt = $l.log
        }
        return
    }

    $arr  = @($l.log)
    $seqs = @($l.seq)
    for ($i = 0; $i -lt $arr.Count; $i++) {
        $s = if ($i -lt $seqs.Count) { [int]$seqs[$i] } else { -1 }
        if ($s -ge 0 -and $s -le $script:lastLogSeq) { continue }
        $txt = [string]$arr[$i]
        if (-not $txt) { continue }
        if ($s -ge 0) { $script:lastLogSeq = $s }
        if ($txt -eq $script:lastLogTxt) { $script:dupCount++; continue }
        if ($script:dupCount -gt 0) {
            Emit ("    …… 相同消息 x{0}" -f $script:dupCount) "DarkGray"
            $script:dupCount = 0
        }
        $script:lastLogTxt = $txt
        $color = "Gray"
        if ($txt -match "堵转|跳闸|失败|出错|错误|超时|冲突") { $color = "Red" }
        elseif ($txt -match "点动|开始|启动|已打开|已保存") { $color = "White" }
        Emit ("[{0}] LOG {1}" -f (TS), $txt) $color
    }
}

# ================= 状态行 + ★电机动作事件 =================
function Show-Status {
    $s = Get-J "/status"
    if ($null -eq $s) { return }

    $md = [int]$s.hardware.motor_direction
    $dirTxt = switch ($md) { 1 { "进料" } -1 { "退料" } default { "停止" } }
    $ma = [int]$s.current.ma
    $pk = [int]$s.current.peak_ma
    $trip = if ($s.current.tripped) { " 堵转LATCH!" } else { "" }

    if ($null -eq $s.feed) {
        $eng = (@($s.hardware.engaged) -join ",")
        $line = "状态: 电机={0} 离合={1} 电流={2}mA 峰值={3}mA act485={4}{5}" -f
            $dirTxt, $(if ($eng) { $eng } else { "-" }), $ma, $pk, $s.act485, $trip
        if ($line -ne $script:lastStatus) {
            Emit ("[{0}] {1}" -f (TS), $line) "Gray"
            $script:lastStatus = $line
        }
        return
    }

    $eng  = [int]$s.feed.clutch_mask
    $pct  = [int]$s.feed.motor_pct
    $actT = [string]$s.feed.motion_text

    $mt = "-"
    if ($null -ne $s.feed.meters) { $mt = "{0:N3}m" -f [double]$s.feed.meters }

    $ph = "-"
    if ($null -ne $s.feed.phase) { $ph = [string]$s.feed.phase }
    $fw = "-"
    if ($null -ne $s.feed.follow) { $fw = [string]$s.feed.follow }

    $sw = "-"
    try {
        $ci = [int]$s.feed.cur
        $chArr = @($s.sensors.ch)
        if ($ci -ge 0 -and $ci -lt $chArr.Count) {
            $mvArr = @($s.sensors.mv)
            $pair = @($chArr[$ci])
            $sw = "S{0}{1}/{2}mV" -f $pair[0], $pair[1], $mvArr[$ci]
        }
    } catch { $sw = "-" }

    # ---------- ★电机动作事件 ----------
    # 只在"真的会动电机"的字段变化时打一行醒目的黄字。
    # 跟随相位(follow)每 250/350ms 跳一次，不单独记事件，只在状态行里显示。
    $evts = @()
    if ($null -ne $script:mPrev.dir -and $dirTxt -ne $script:mPrev.dir) {
        $evts += ("电机 {0} -> {1}" -f $script:mPrev.dir, $dirTxt)
    }
    if ($script:mPrev.eng -ge 0 -and $eng -ne $script:mPrev.eng) {
        $evts += ("离合 0x{0:X2} -> 0x{1:X2}" -f $script:mPrev.eng, $eng)
    }
    if ($script:mPrev.act -ne "" -and $actT -ne $script:mPrev.act) {
        $evts += ("动作 {0} -> {1}" -f $script:mPrev.act, $actT)
    }
    if ($script:mPrev.phase -ne "" -and $ph -ne $script:mPrev.phase) {
        $evts += ("阶段 {0} -> {1}" -f $script:mPrev.phase, $ph)
    }
    if ($script:mPrev.pct -ge 0 -and $pct -ne $script:mPrev.pct -and $dirTxt -ne "停止") {
        $evts += ("占空比 {0}% -> {1}%" -f $script:mPrev.pct, $pct)
    }
    if ($evts.Count -gt 0) {
        Emit ("[{0}] ★电机 {1}" -f (TS), ($evts -join " | ")) "Yellow"
    }
    $script:mPrev.dir   = $dirTxt
    $script:mPrev.eng   = $eng
    $script:mPrev.act   = $actT
    $script:mPrev.phase = $ph
    $script:mPrev.pct   = $pct

    $line = "状态: 电机={0}/{1}% 离合=0x{2:X2} 电流={3}mA 峰值={4}mA act485={5}{6} | 料位{7} 动作={8} 在线={9} 打印={10} 堵转={11} 点动={12} 回读={13} cmd={14} 计米={15} 阶段={16} 跟随={17} 微动={18}" -f
        $dirTxt, $pct, $eng, $ma, $pk, $s.act485, $trip,
        $s.feed.cur, $actT, $s.feed.online, $s.feed.printing,
        $s.feed.tripped, $s.feed.jog, $s.feed.clutch_rb, $s.feed.cmd, $mt,
        $ph, $fw, $sw

    if ($line -ne $script:lastStatus) {
        Emit ("[{0}] {1}" -f (TS), $line) "Gray"
        $script:lastStatus = $line
    }
}

# ---------- 启动横幅 ----------
Emit ("=" * 78) "DarkGray"
Emit ("485 通讯监控  ->  $base    轮询 ${iv}ms（抓帧优先，/log 与 /status 每 400ms 一次）") "Cyan"
if ($logFile) {
    Emit ("完整日志文件：$logFile") "Cyan"
    Emit ("  （控制台滚掉的内容都在文件里；停止后直接用记事本/VS Code 打开）") "DarkGray"
} else {
    Emit ("[!] 未启用日志文件，控制台滚掉就没了") "Yellow"
}
Emit ("  [RX 青] 打印机->本机   [TX 绿] 本机->打印机   [灰] WEB日志/状态   [黄] ★电机动作   [红] 丢帧告警") "DarkGray"
Emit ("  Ctrl+C 停止") "DarkGray"
Emit ("=" * 78) "DarkGray"

$startAt    = Get-Date
$lastSlowAt = Get-Date

try {
    while ($true) {
        $cycStart = Get-Date

        # ================= 485 原始报文（内层追平，一次拉干净） =================
        $pulls = 0
        while ($pulls -lt 12) {
            $d = Get-J "/bus485raw?since=$lastSeq"
            if ($null -eq $d) {
                $rawBad++
                if (-not $rawGone) {
                    Emit "[!] /bus485raw 取不到：固件需要 v1.1.3+（当前设备可能还是旧版）" "Yellow"
                    Emit "    也可以先在浏览器打开 http://$ip/bus485raw 手测一下。" "Yellow"
                    $rawGone = $true
                }
                break
            }
            if ($rawBad) { Emit "485 通道恢复" "Gray"; $rawBad = 0 }

            $got = 0
            $devLast = 0
            foreach ($ln in ($d -split "`n")) {
                $ln = $ln.Trim()
                if (-not $ln) { continue }

                if ($ln -match '^(\d+) (\d+) (RX|TX) (\d+) ([0-9A-Fa-f]*)$') {
                    $seq = [int]$Matches[1]
                    $lastSeq = $seq
                    $t   = $Matches[2]
                    $dir = $Matches[3]
                    $len = $Matches[4]
                    $hex = $Matches[5]
                    $got++
                    $frameTotal++

                    # ---------- 丢帧检测 ----------
                    # 设备端环形缓冲只有 32 帧，轮询稍慢就会被新帧盖掉。
                    # 靠 seq 是否连续来判定：断了就明确告警，不用靠猜。
                    if ($seqPrev -ge 0 -and $seq -gt ($seqPrev + 1)) {
                        $lost = $seq - $seqPrev - 1
                        $lostTotal += $lost
                        Emit ("[{0}] ⚠ 丢帧 {1} 条（seq {2} -> {3}）" -f (TS), $lost, $seqPrev, $seq) "Red"
                    }
                    $seqPrev = $seq

                    $mark = ""
                    if ($hex.Length -ge 2 -and $hex.Substring(0, 2).ToUpper() -eq "3D") { $mark = " [3D]" }
                    $dec = Decode-Frame $hex

                    if ($dir -eq "RX") {
                        Emit ("[{0}] RX #{1} t={2} len={3}{4} {5}{6}" -f (TS), $seq, $t, $len, $mark, $hex, $dec) "Cyan"
                    } else {
                        Emit ("[{0}] TX #{1} t={2} len={3}{4} {5}{6}" -f (TS), $seq, $t, $len, $mark, $hex, $dec) "Green"
                    }
                } elseif ($ln -match 'LAST_SEQ=(\d+)') {
                    $devLast = [int]$Matches[1]
                    if ($ln -ne $lastSum) { Emit ("  " + $ln) "DarkGray"; $lastSum = $ln }
                }
            }

            $pulls++
            if ($got -eq 0) { break }                              # 没新帧，退出内层
            if ($devLast -gt 0 -and $lastSeq -ge $devLast) { break } # 已追平设备端最新，退出内层
        }

        # ================= 慢通道：/log 与 /status（400ms 一次） =================
        # 放低频是刻意的：它们每次 HTTP 往返都要 20~40ms，占了空档就会丢帧。
        $now = Get-Date
        if (($now - $lastSlowAt).TotalMilliseconds -ge 400) {
            $lastSlowAt = $now
            Show-Log
            Show-Status
        }

        # ================= 节流 =================
        # 抓帧优先：只要还有帧在来，就尽快回到下一轮拉取，只留 20ms 喘息。
        $el = ((Get-Date) - $cycStart).TotalMilliseconds
        $wait = [int]($iv - $el)
        if ($wait -lt 20) { $wait = 20 }
        Start-Sleep -Milliseconds $wait
    }
} finally {
    $dur = (Get-Date) - $startAt
    Emit ("") "Gray"
    Emit ("监控停止：运行 {0:hh\:mm\:ss}，共收到 {1} 帧，累计丢帧 {2} 条" -f $dur, $frameTotal, $lostTotal) "Cyan"
    if ($lostTotal -gt 0) {
        Emit ("  丢帧说明：设备端 485 环形缓冲只有 32 帧，轮询跟不上就会被覆盖。") "Yellow"
        Emit ("  对策：mon485.bat 192.168.2.100 20  （间隔改 20ms），或关掉别的页面/脚本再抓。") "Yellow"
    } else {
        Emit ("  无丢帧，日志完整。") "Green"
    }
    if ($logFile) { Emit ("完整日志：$logFile") "Cyan" }
    if ($null -ne $logSw) {
        try { $logSw.Flush(); $logSw.Close() } catch { }
        $logSw = $null
    }
}
