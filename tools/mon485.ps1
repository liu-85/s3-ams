# mon485.ps1 —— 485 通讯监控（打印机原始指令 + 本机回复 + WEB日志）
# 由 mon485.bat 启动；参数经环境变量传入：AMS_IP / AMS_MS
#   [RX 青色] 打印机 -> 本机    [TX 绿色] 本机 -> 打印机    [灰] WEB日志/状态

$ErrorActionPreference = 'Stop'
$ip = $env:AMS_IP; if (-not $ip) { $ip = "192.168.2.100" }
$iv = $env:AMS_MS; if (-not $iv) { $iv = "300" }
$iv = [int]$iv
$base = "http://$ip"

function Get-J($path) {
    try { return Invoke-RestMethod -Uri ($base + $path) -TimeoutSec 3 -UseBasicParsing } catch { return $null }
}
function TS { Get-Date -Format "HH:mm:ss" }

Write-Host "=== 485 通讯监控 -> $base （轮询 ${iv}ms）===" -ForegroundColor Cyan
Write-Host "  [RX 青色] 打印机 -> 本机    [TX 绿色] 本机 -> 打印机    [灰] WEB日志/状态" -ForegroundColor DarkGray
Write-Host "  Ctrl+C 停止`n" -ForegroundColor DarkGray

$lastSeq    = 0        # 485 帧序号游标
$lastLogSeq = -1       # WEB 日志行号游标
$lastLogTxt = ""       # 上一条 WEB 日志文本（连续相同则折叠）
$dupCount   = 0        # 连续相同折叠计数
$lastStatus = ""       # 上一条状态行（没变化不刷新）
$lastSum    = ""       # 上一条汇总行
$rawBad     = 0
$rawGone    = $false

try {
    while ($true) {
        # ---------- 485 原始报文 ----------
        $d = Get-J "/bus485raw?since=$lastSeq"
        if ($null -eq $d) {
            $rawBad++
            if (-not $rawGone) {
                Write-Host "[!] /bus485raw 取不到：固件需要 v1.1.3+（当前设备可能还是旧版）" -ForegroundColor Yellow
                Write-Host "    先刷 release\v1.1.3 再开本脚本；本提示只出一次。`n" -ForegroundColor Yellow
                $rawGone = $true
            }
        } else {
            if ($rawBad) { Write-Host "485 通道恢复`n" -ForegroundColor Gray; $rawBad = 0 }
            foreach ($ln in ($d -split "`n")) {
                $ln = $ln.Trim()
                if (-not $ln) { continue }
                if ($ln -match '^(\d+) (\d+) (RX|TX) (\d+) ([0-9A-Fa-f]*)$') {
                    $lastSeq = [int]$Matches[1]
                    $t   = $Matches[2]
                    $dir = $Matches[3]
                    $len = $Matches[4]
                    $hex = $Matches[5]
                    $mark = ""
                    if ($hex.Length -ge 2 -and $hex.Substring(0,2).ToUpper() -eq "3D") { $mark = " [帧头3D]" }
                    if ($dir -eq "RX") {
                        Write-Host ("[{0}] RX t={1} len={2}{3} {4}" -f (TS), $t, $len, $mark, $hex) -ForegroundColor Cyan
                    } else {
                        Write-Host ("[{0}] TX t={1} len={2}{3} {4}" -f (TS), $t, $len, $mark, $hex) -ForegroundColor Green
                    }
                } elseif ($ln -match '^(RX_TOTAL|TX_TOTAL|LAST_SEQ)') {
                    if ($ln -ne $lastSum) { Write-Host ("  " + $ln) -ForegroundColor DarkGray; $lastSum = $ln }
                }
            }
        }

        # ---------- WEB 日志（新行才出；连续相同的折叠成 xN） ----------
        $l = Get-J "/log"
        if ($null -ne $l) {
            if ($l.log -is [string]) {
                # 旧固件：整个是一串，只在内容变化时提示一次
                if ($l.log -and $l.log -ne $lastLogTxt) {
                    Write-Host ("[{0}] LOG(旧格式) {1}" -f (TS), $l.log) -ForegroundColor Gray
                    $lastLogTxt = $l.log
                }
            } else {
                $arr  = @($l.log)
                $seqs = @($l.seq)
                for ($i = 0; $i -lt $arr.Count; $i++) {
                    $s = if ($i -lt $seqs.Count) { [int]$seqs[$i] } else { -1 }
                    if ($s -ge 0 -and $s -le $lastLogSeq) { continue }
                    $txt = [string]$arr[$i]
                    if (-not $txt) { continue }
                    if ($s -ge 0) { $lastLogSeq = $s }
                    if ($txt -eq $lastLogTxt) {
                        $dupCount++          # 同样的消息没变化：不更新，只计数
                        continue
                    }
                    if ($dupCount -gt 0) {
                        Write-Host ("    …… 相同消息 x{0}" -f $dupCount) -ForegroundColor DarkGray
                        $dupCount = 0
                    }
                    $lastLogTxt = $txt
                    $color = "Gray"
                    if ($txt -match "堵转|跳闸|失败|出错|错误|超时|冲突") { $color = "Red" }
                    elseif ($txt -match "点动|开始|启动|已打开|已保存") { $color = "White" }
                    Write-Host ("[{0}] LOG {1}" -f (TS), $txt) -ForegroundColor $color
                }
            }
        }

        # ---------- 状态行（没变化不刷新） ----------
        $s = Get-J "/status"
        if ($null -ne $s) {
            $md = [int]$s.hardware.motor_direction
            $dirTxt = switch ($md) { 1 { "进料" } -1 { "退料" } default { "停止" } }
            $ma = [int]$s.current.ma
            $pk = [int]$s.current.peak_ma
            $trip = if ($s.current.tripped) { " 堵转LATCH!" } else { "" }
            $eng = (@($s.hardware.engaged) -join ",")
            $line = "状态: 电机={0} 离合={1} 电流={2}mA 峰值={3}mA act485={4}{5}" -f
                $dirTxt, $(if ($eng) { $eng } else { "-" }), $ma, $pk, $s.act485, $trip
            if ($line -ne $lastStatus) {
                Write-Host ("[{0}] {1}" -f (TS), $line) -ForegroundColor Gray
                $lastStatus = $line
            }
        }

        Start-Sleep -Milliseconds $iv
    }
} finally {
    Write-Host "`n监控停止" -ForegroundColor Cyan
}
