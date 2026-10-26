# 使用系统微软雅黑生成原生 16×16 菜单点阵；正常编译无需运行此脚本。
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing
$labels = @('循迹菜单', '电感标定', 'PID调参', '开始运行', '舵机中值', '舵机左限位', '舵机右限位', '居中标定', '速度设定', '恢复PID初值', '标定设置', '舵机设置', '电机设置', '舵机方向', '左电机方向', '右电机方向', '正向', '反向', '速度PID', '动力限幅', '转向PID调参', '速度闭环调参')
$chars = [System.Collections.Generic.List[char]]::new()
foreach ($label in $labels) { foreach ($ch in $label.ToCharArray()) { if (-not $chars.Contains($ch)) { $chars.Add($ch) } } }
$font = [System.Drawing.Font]::new('Microsoft YaHei', 14, [System.Drawing.FontStyle]::Regular, [System.Drawing.GraphicsUnit]::Pixel)
$bitmap = [System.Drawing.Bitmap]::new(16,16)
$graphics = [System.Drawing.Graphics]::FromImage($bitmap)
$graphics.TextRenderingHint = [System.Drawing.Text.TextRenderingHint]::SingleBitPerPixelGridFit
$format = [System.Drawing.StringFormat]::GenericTypographic.Clone()
$format.FormatFlags = $format.FormatFlags -bor [System.Drawing.StringFormatFlags]::NoWrap
$lines = [System.Collections.Generic.List[string]]::new()
$lines.Add('/* 由 tools/generate_fonts.ps1 生成；OLED 竖向页格式，请修改生成脚本而非手改点阵。 */')
$lines.Add('#ifndef TRACK_FONT_H')
$lines.Add('#define TRACK_FONT_H')
$lines.Add(('static const unsigned char track_glyphs[{0}][32] = {{' -f $chars.Count))
foreach ($ch in $chars) {
    $graphics.Clear([System.Drawing.Color]::Black)
    $graphics.DrawString([string]$ch, $font, [System.Drawing.Brushes]::White, 0, -1, $format)
    $bytes = [System.Collections.Generic.List[string]]::new()
    for ($page=0; $page -lt 2; $page++) { for ($x=0; $x -lt 16; $x++) {
        $value=0
        for ($bit=0; $bit -lt 8; $bit++) { if ($bitmap.GetPixel($x, $page*8+$bit).R -gt 127) { $value = $value -bor (1 -shl $bit) } }
        $bytes.Add(('0x{0:X2}' -f $value))
    } }
    $lines.Add(' {' + ($bytes -join ',') + '}, /* U+' + ('{0:X4}' -f [int]$ch) + ' */')
}
$lines.Add('};')
$lines.Add(('static const unsigned char track_label_ids[{0}][9] = {{' -f $labels.Count))
foreach ($label in $labels) {
    $ids = @($label.ToCharArray() | ForEach-Object { $chars.IndexOf($_) }) + @(255)
    $lines.Add(' {' + ($ids -join ',') + '},')
}
$lines.Add('};')
$lines.Add('#endif')
$target = Join-Path $PSScriptRoot '..\USER\inc\track_font.h'
[System.IO.File]::WriteAllLines($target, $lines, [System.Text.UTF8Encoding]::new($false))
$preview = [System.Drawing.Bitmap]::new(128,($labels.Count*18))
$canvas = [System.Drawing.Graphics]::FromImage($preview)
$canvas.Clear([System.Drawing.Color]::Black)
$canvas.TextRenderingHint = [System.Drawing.Text.TextRenderingHint]::SingleBitPerPixelGridFit
for ($n=0; $n -lt $labels.Count; $n++) { $canvas.DrawString($labels[$n],$font,[System.Drawing.Brushes]::White,0,$n*18,$format) }
$preview.Save((Join-Path $PSScriptRoot '..\MDK\menu-font-preview.png'))
$canvas.Dispose(); $preview.Dispose(); $format.Dispose(); $graphics.Dispose(); $bitmap.Dispose(); $font.Dispose()
Write-Output ('Generated {0} glyphs and a menu preview.' -f $chars.Count)
