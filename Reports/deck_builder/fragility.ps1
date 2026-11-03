param($pptx, $outdir)
# Fragility check in PowerPoint itself (window open, so layout is computed as in the editor):
#  1. every text shape: text taller than its box (overflows now), or box off the slide
#  2. every text shape's auto-fit mode (2 = shrink on overflow; 1 = grows the shape)
#  3. simulated edits on the new / changed slides, exported for a look
$pptx = (Resolve-Path $pptx).Path
New-Item -ItemType Directory -Force $outdir | Out-Null
$app = New-Object -ComObject PowerPoint.Application
$p = $app.Presentations.Open($pptx, -1, 0, -1)
$W = $p.PageSetup.SlideWidth; $H = $p.PageSetup.SlideHeight
$modes = @{}
function Check($sl, $s) {
  if (-not $s.HasTextFrame) { return }
  $tf = $s.TextFrame2
  if (-not $tf.HasText) { return }
  $isPh = ($s.Type -eq 14)
  if (-not $isPh) { $script:modes[[string]$tf.AutoSize] += 1 }
  $bh = $tf.TextRange.BoundHeight + $tf.MarginTop + $tf.MarginBottom
  if ($bh -gt $s.Height + 3) { "OVERFLOW slide {0} '{1}': text {2:N0} pt > box {3:N0} pt  [{4}]" -f $sl, $s.Name, $bh, $s.Height, $tf.TextRange.Text.Substring(0, [Math]::Min(40, $tf.TextRange.Text.Length)) }
  if ($s.Left -lt -1 -or $s.Top -lt -1 -or $s.Left + $s.Width -gt $W + 1 -or $s.Top + $s.Height -gt $H + 1) { "OFFSLIDE slide {0} '{1}'" -f $sl, $s.Name }
}
foreach ($sl in $p.Slides) {
  $p.Windows(1).View.GotoSlide($sl.SlideIndex)
  foreach ($s in $sl.Shapes) {
    if ($s.Type -eq 6) { foreach ($g in $s.GroupItems) { Check $sl.SlideIndex $g } } else { Check $sl.SlideIndex $s }
  }
}
"autofit modes (non-placeholder text shapes): " + (($modes.GetEnumerator() | ForEach-Object { "$($_.Key)=$($_.Value)" }) -join ', ')
function Find-Shape($slide, $start) {
  foreach ($s in $slide.Shapes) {
    if ($s.Type -eq 6) { foreach ($g in $s.GroupItems) { if ($g.HasTextFrame -and $g.TextFrame.TextRange.Text.StartsWith($start)) { return $g } } }
    elseif ($s.HasTextFrame -and $s.TextFrame.TextRange.Text.StartsWith($start)) { return $s }
  }
}
# edits: table cell (3), caption + facts (10), MEASURE block (11), results column (19)
foreach ($s in $p.Slides(3).Shapes) { if ($s.HasTable) { $s.Table.Cell(2, 3).Shape.TextFrame.TextRange.InsertAfter(" Added note on the measurement.") | Out-Null } }
$c = Find-Shape $p.Slides(10) 'Printed battery'; $c.TextFrame.TextRange.InsertAfter(" (two packs per side)") | Out-Null
$f = Find-Shape $p.Slides(10) 'Frame'; $f.TextFrame.TextRange.InsertAfter("`rSpare: second Teensy fitted 10-30") | Out-Null
$m = Find-Shape $p.Slides(11) 'MEASURE'; $m.TextFrame.TextRange.InsertAfter("`rEXTRA: an added sentence to see what happens when the block gets longer than its space.") | Out-Null
$r = Find-Shape $p.Slides(19) 'Req. 1'; if ($r) { $r.TextFrame.TextRange.InsertAfter("`rAdded item one`rAdded item two") | Out-Null }
foreach ($i in 3, 10, 11, 19) { $p.Windows(1).View.GotoSlide($i); Start-Sleep -Milliseconds 300; $p.Slides($i).Export("$outdir\edit_$i.png", "PNG", 1280, 960) }
$p.Close()
$app.Quit()
"done"
