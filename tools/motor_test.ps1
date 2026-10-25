# motor_test.ps1 —— 电机检测（进料/退料点动 + 电流/方向判定）
# 由 motor_test.bat 启动；参数经环境变量传入：AMS_IP / AMS_CH / AMS_SEC / AMS_PCT

$ErrorActionPreference = 'Stop'
$ip  = $env:AMS_IP;  if (-not $ip)  { $ip = "192.168.2.100" }
$ch  = $env:AMS_CH;  if (-not $ch)  { $ch = "1" }
$sec = $env:AMS_SEC; if (-not $sec) { $sec = "1.5" }
$pct = $env:AMS_PCT; if (-not $pct) { $pct = "100" }
$ch  = [int]$ch
$sec = [double]$sec
$pct = [int]$pct
$base = "http://$ip"

function Get-J($path) {
    try { return Invoke-RestMethod -Uri ($base + $path) -TimeoutSec 3 -UseBasicParsing } catch { return $null }
}
function Post-J($path, $obj) {
    try {
        # 必须 JSON：固件各接口读的是 server.arg("plain")（原始 body），
        # 默认表单编码会进 named args 而不是 plain -> 400 no body
        return Invoke-RestMethod -Uri ($base + $path) -Method Post `
            -ContentType "application/json" -Body ($obj | ConvertTo-Json) `
            -TimeoutSec 3 -UseBasicParsing
    } catch { return $null }
}
function TS { Get-Date -Format "HH:mm:ss" }

Write-Host "=== 电机检测 -> $base ===" -ForegroundColor Cyan
Write-Host ("通道 {0} · 每次 {1}s · PWM {2}% (duty {3}/255)" -f $ch, $sec, $pct, [math]::Round($pct*255/100)) -ForegroundColor Cyan

$st = Get-J "/status"
if (-not $st) { Write-Host "连不上设备 $base（先确认 IP/网页能打开）" -ForegroundColor Red; exit 1 }
Write-Host ("固件 {0} · 设备点动PWM {1}% · act485 {2} · 电流采样 {3}" -f
    $st.app_version, $st.jog_speed_pct, $st.act485,
    $(if ($st.current.ready) { "就绪" } else { "不可用" })) -ForegroundColor DarkGray
if (-not $st.current.ready) {
    Write-Host "电流采样不可用：转动判定只靠电机方向，看不到电流佐证" -ForegroundColor Yellow
}

$r = Post-J "/jog_set" @{ seconds = $sec; speed_pct = $pct }
if (-not $r -or -not $r.ok) { Write-Host "设置点动参数失败" -ForegroundColor Red; exit 1 }

function Run-Jog([int]$dir, [string]$name) {
    $dur = [int]($sec * 1000) + 900
    $sw  = [Diagnostics.Stopwatch]::StartNew()
    $req = Post-J "/hardware_test" @{ channel = $ch; direction = $dir }
    if (-not $req -or -not $req.ok) {
        Write-Host ("[{0}] {1} 点动启动失败：{2}" -f (TS), $name, $(if ($req) { $req.info } else { "无响应" })) -ForegroundColor Red
        return
    }
    Write-Host ("[{0}] {1} 开始（{2}）" -f (TS), $name, $req.info) -ForegroundColor Cyan

    $maxMa = 0; $sawDir = $false; $eng = ""; $last = ""
    while ($sw.ElapsedMilliseconds -lt $dur) {
        $s = Get-J "/status"
        if ($s) {
            $md = [int]$s.hardware.motor_direction
            $ma = [int]$s.current.ma
            if ($ma -gt $maxMa) { $maxMa = $ma }
            if ($md -eq $dir) { $sawDir = $true }
            $e = (@($s.hardware.engaged) -join ",")
            if ($e) { $eng = $e }
            $line = "    dir={0} 离合={1} 电流={2}mA 峰值={3}mA" -f $md, $(if ($e) { $e } else { "-" }), $ma, $maxMa
            if ($line -ne $last) { Write-Host $line -ForegroundColor DarkGray; $last = $line }
        }
        Start-Sleep -Milliseconds 150
    }
    $s = Get-J "/status"
    $maNow = if ($s) { [int]$s.current.ma } else { -1 }

    if ($sawDir -and $maxMa -ge 150) {
        Write-Host ("=> {0}：转（方向到位，峰值 {1}mA，停后 {2}mA）" -f $name, $maxMa, $maNow) -ForegroundColor Green
    } elseif ($sawDir -and $maxMa -lt 150) {
        Write-Host ("=> {0}：方向给了但电流只有 {1}mA —— PWM 没推动电机" -f $name, $maxMa) -ForegroundColor Yellow
        Write-Host "    查：离合是否吸合（日志有离合行）· 占空比是否过低（往 100% 加）· H 桥/接线" -ForegroundColor Yellow
    } else {
        Write-Host ("=> {0}：没转（电机方向始终没变成 {1}，最大电流 {2}mA）" -f $name, $dir, $maxMa) -ForegroundColor Red
        Write-Host "    查：act485 是否开 · 固件版本（>=1.1.3 修了离合咬合后电机被停的 bug）· 离合/电机接线" -ForegroundColor Red
    }
}

try {
    Run-Jog  1 "进料"
    Start-Sleep -Milliseconds 600
    Run-Jog -1 "退料"
} finally {
    Write-Host ""
    Write-Host "=== 检测完成 ===" -ForegroundColor Cyan
    Write-Host "提示：都没转但网页日志有点动行 -> 硬件/接线；日志连点动行都没有 -> 固件或接口问题" -ForegroundColor DarkGray
}
