pio run -t clean -e xiao-esp32s3
pio run -e xiao-esp32s3 -t upload; if ($LASTEXITCODE -eq 0) { pio device monitor }
