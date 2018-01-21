These videos were converted with:

````
ffmpeg.exe -i .\doc_2018-01-21_17-39-33.mp4 -filter:v "crop=320:240:20:170"  -movflags faststart -pix_fmt yuv420p -r 20 -an -preset veryslow  -y out.mp4
ffmpeg.exe -i .\toboot-mode.mp4 -vcodec libvpx -an .\toboot-mode.webm
ffmpeg.exe -i .\toboot-mode.mp4 -c:v libtheora -an -q:v 10 .\toboot-mode.ogv

ffmpeg -y -i .\toboot-mode.mp4 -vf fps=10,scale=320:-1:flags=lanczos,palettegen palette.png
 ffmpeg.exe -i .\toboot-mode.mp4 -i .\palette.png -filter_complex "fps=10,scale=320:-1:flags=lanczos[x];[x][1:v]paletteuse" .\toboot-mode.gif
````