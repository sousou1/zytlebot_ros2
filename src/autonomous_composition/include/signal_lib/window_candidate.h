int window_num = 364;
int w[364][2][2] = {
{{ 24 , 64 },{ 160 , 240 }},
{{ 24 , 64 },{ 200 , 280 }},
{{ 24 , 64 },{ 240 , 320 }},
{{ 24 , 64 },{ 280 , 360 }},
{{ 24 , 64 },{ 320 , 400 }},
{{ 24 , 64 },{ 360 , 440 }},
{{ 24 , 64 },{ 400 , 480 }},
{{ 44 , 84 },{ 160 , 240 }},
{{ 44 , 84 },{ 200 , 280 }},
{{ 44 , 84 },{ 240 , 320 }},
{{ 44 , 84 },{ 280 , 360 }},
{{ 44 , 84 },{ 320 , 400 }},
{{ 44 , 84 },{ 360 , 440 }},
{{ 44 , 84 },{ 400 , 480 }},
{{ 64 , 104 },{ 160 , 240 }},
{{ 64 , 104 },{ 200 , 280 }},
{{ 64 , 104 },{ 240 , 320 }},
{{ 64 , 104 },{ 280 , 360 }},
{{ 64 , 104 },{ 320 , 400 }},
{{ 64 , 104 },{ 360 , 440 }},
{{ 64 , 104 },{ 400 , 480 }},
{{ 84 , 124 },{ 160 , 240 }},
{{ 84 , 124 },{ 200 , 280 }},
{{ 84 , 124 },{ 240 , 320 }},
{{ 84 , 124 },{ 280 , 360 }},
{{ 84 , 124 },{ 320 , 400 }},
{{ 84 , 124 },{ 360 , 440 }},
{{ 84 , 124 },{ 400 , 480 }},
{{ 104 , 144 },{ 160 , 240 }},
{{ 104 , 144 },{ 200 , 280 }},
{{ 104 , 144 },{ 240 , 320 }},
{{ 104 , 144 },{ 280 , 360 }},
{{ 104 , 144 },{ 320 , 400 }},
{{ 104 , 144 },{ 360 , 440 }},
{{ 104 , 144 },{ 400 , 480 }},
{{ 124 , 164 },{ 160 , 240 }},
{{ 124 , 164 },{ 200 , 280 }},
{{ 124 , 164 },{ 240 , 320 }},
{{ 124 , 164 },{ 280 , 360 }},
{{ 124 , 164 },{ 320 , 400 }},
{{ 124 , 164 },{ 360 , 440 }},
{{ 124 , 164 },{ 400 , 480 }},
{{ 144 , 184 },{ 160 , 240 }},
{{ 144 , 184 },{ 200 , 280 }},
{{ 144 , 184 },{ 240 , 320 }},
{{ 144 , 184 },{ 280 , 360 }},
{{ 144 , 184 },{ 320 , 400 }},
{{ 144 , 184 },{ 360 , 440 }},
{{ 144 , 184 },{ 400 , 480 }},
{{ 164 , 204 },{ 160 , 240 }},
{{ 164 , 204 },{ 200 , 280 }},
{{ 164 , 204 },{ 240 , 320 }},
{{ 164 , 204 },{ 280 , 360 }},
{{ 164 , 204 },{ 320 , 400 }},
{{ 164 , 204 },{ 360 , 440 }},
{{ 164 , 204 },{ 400 , 480 }},
{{ 184 , 224 },{ 160 , 240 }},
{{ 184 , 224 },{ 200 , 280 }},
{{ 184 , 224 },{ 240 , 320 }},
{{ 184 , 224 },{ 280 , 360 }},
{{ 184 , 224 },{ 320 , 400 }},
{{ 184 , 224 },{ 360 , 440 }},
{{ 184 , 224 },{ 400 , 480 }},
{{ 204 , 244 },{ 160 , 240 }},
{{ 204 , 244 },{ 200 , 280 }},
{{ 204 , 244 },{ 240 , 320 }},
{{ 204 , 244 },{ 280 , 360 }},
{{ 204 , 244 },{ 320 , 400 }},
{{ 204 , 244 },{ 360 , 440 }},
{{ 204 , 244 },{ 400 , 480 }},
{{ 224 , 264 },{ 160 , 240 }},
{{ 224 , 264 },{ 200 , 280 }},
{{ 224 , 264 },{ 240 , 320 }},
{{ 224 , 264 },{ 280 , 360 }},
{{ 224 , 264 },{ 320 , 400 }},
{{ 224 , 264 },{ 360 , 440 }},
{{ 224 , 264 },{ 400 , 480 }},
{{ 244 , 284 },{ 160 , 240 }},
{{ 244 , 284 },{ 200 , 280 }},
{{ 244 , 284 },{ 240 , 320 }},
{{ 244 , 284 },{ 280 , 360 }},
{{ 244 , 284 },{ 320 , 400 }},
{{ 244 , 284 },{ 360 , 440 }},
{{ 244 , 284 },{ 400 , 480 }},
{{ 264 , 304 },{ 160 , 240 }},
{{ 264 , 304 },{ 200 , 280 }},
{{ 264 , 304 },{ 240 , 320 }},
{{ 264 , 304 },{ 280 , 360 }},
{{ 264 , 304 },{ 320 , 400 }},
{{ 264 , 304 },{ 360 , 440 }},
{{ 264 , 304 },{ 400 , 480 }},
{{ 24 , 84 },{ 160 , 280 }},
{{ 24 , 84 },{ 190 , 310 }},
{{ 24 , 84 },{ 220 , 340 }},
{{ 24 , 84 },{ 250 , 370 }},
{{ 24 , 84 },{ 280 , 400 }},
{{ 24 , 84 },{ 310 , 430 }},
{{ 24 , 84 },{ 340 , 460 }},
{{ 24 , 84 },{ 370 , 490 }},
{{ 24 , 84 },{ 400 , 520 }},
{{ 39 , 99 },{ 160 , 280 }},
{{ 39 , 99 },{ 190 , 310 }},
{{ 39 , 99 },{ 220 , 340 }},
{{ 39 , 99 },{ 250 , 370 }},
{{ 39 , 99 },{ 280 , 400 }},
{{ 39 , 99 },{ 310 , 430 }},
{{ 39 , 99 },{ 340 , 460 }},
{{ 39 , 99 },{ 370 , 490 }},
{{ 39 , 99 },{ 400 , 520 }},
{{ 54 , 114 },{ 160 , 280 }},
{{ 54 , 114 },{ 190 , 310 }},
{{ 54 , 114 },{ 220 , 340 }},
{{ 54 , 114 },{ 250 , 370 }},
{{ 54 , 114 },{ 280 , 400 }},
{{ 54 , 114 },{ 310 , 430 }},
{{ 54 , 114 },{ 340 , 460 }},
{{ 54 , 114 },{ 370 , 490 }},
{{ 54 , 114 },{ 400 , 520 }},
{{ 69 , 129 },{ 160 , 280 }},
{{ 69 , 129 },{ 190 , 310 }},
{{ 69 , 129 },{ 220 , 340 }},
{{ 69 , 129 },{ 250 , 370 }},
{{ 69 , 129 },{ 280 , 400 }},
{{ 69 , 129 },{ 310 , 430 }},
{{ 69 , 129 },{ 340 , 460 }},
{{ 69 , 129 },{ 370 , 490 }},
{{ 69 , 129 },{ 400 , 520 }},
{{ 84 , 144 },{ 160 , 280 }},
{{ 84 , 144 },{ 190 , 310 }},
{{ 84 , 144 },{ 220 , 340 }},
{{ 84 , 144 },{ 250 , 370 }},
{{ 84 , 144 },{ 280 , 400 }},
{{ 84 , 144 },{ 310 , 430 }},
{{ 84 , 144 },{ 340 , 460 }},
{{ 84 , 144 },{ 370 , 490 }},
{{ 84 , 144 },{ 400 , 520 }},
{{ 99 , 159 },{ 160 , 280 }},
{{ 99 , 159 },{ 190 , 310 }},
{{ 99 , 159 },{ 220 , 340 }},
{{ 99 , 159 },{ 250 , 370 }},
{{ 99 , 159 },{ 280 , 400 }},
{{ 99 , 159 },{ 310 , 430 }},
{{ 99 , 159 },{ 340 , 460 }},
{{ 99 , 159 },{ 370 , 490 }},
{{ 99 , 159 },{ 400 , 520 }},
{{ 114 , 174 },{ 160 , 280 }},
{{ 114 , 174 },{ 190 , 310 }},
{{ 114 , 174 },{ 220 , 340 }},
{{ 114 , 174 },{ 250 , 370 }},
{{ 114 , 174 },{ 280 , 400 }},
{{ 114 , 174 },{ 310 , 430 }},
{{ 114 , 174 },{ 340 , 460 }},
{{ 114 , 174 },{ 370 , 490 }},
{{ 114 , 174 },{ 400 , 520 }},
{{ 129 , 189 },{ 160 , 280 }},
{{ 129 , 189 },{ 190 , 310 }},
{{ 129 , 189 },{ 220 , 340 }},
{{ 129 , 189 },{ 250 , 370 }},
{{ 129 , 189 },{ 280 , 400 }},
{{ 129 , 189 },{ 310 , 430 }},
{{ 129 , 189 },{ 340 , 460 }},
{{ 129 , 189 },{ 370 , 490 }},
{{ 129 , 189 },{ 400 , 520 }},
{{ 144 , 204 },{ 160 , 280 }},
{{ 144 , 204 },{ 190 , 310 }},
{{ 144 , 204 },{ 220 , 340 }},
{{ 144 , 204 },{ 250 , 370 }},
{{ 144 , 204 },{ 280 , 400 }},
{{ 144 , 204 },{ 310 , 430 }},
{{ 144 , 204 },{ 340 , 460 }},
{{ 144 , 204 },{ 370 , 490 }},
{{ 144 , 204 },{ 400 , 520 }},
{{ 159 , 219 },{ 160 , 280 }},
{{ 159 , 219 },{ 190 , 310 }},
{{ 159 , 219 },{ 220 , 340 }},
{{ 159 , 219 },{ 250 , 370 }},
{{ 159 , 219 },{ 280 , 400 }},
{{ 159 , 219 },{ 310 , 430 }},
{{ 159 , 219 },{ 340 , 460 }},
{{ 159 , 219 },{ 370 , 490 }},
{{ 159 , 219 },{ 400 , 520 }},
{{ 174 , 234 },{ 160 , 280 }},
{{ 174 , 234 },{ 190 , 310 }},
{{ 174 , 234 },{ 220 , 340 }},
{{ 174 , 234 },{ 250 , 370 }},
{{ 174 , 234 },{ 280 , 400 }},
{{ 174 , 234 },{ 310 , 430 }},
{{ 174 , 234 },{ 340 , 460 }},
{{ 174 , 234 },{ 370 , 490 }},
{{ 174 , 234 },{ 400 , 520 }},
{{ 189 , 249 },{ 160 , 280 }},
{{ 189 , 249 },{ 190 , 310 }},
{{ 189 , 249 },{ 220 , 340 }},
{{ 189 , 249 },{ 250 , 370 }},
{{ 189 , 249 },{ 280 , 400 }},
{{ 189 , 249 },{ 310 , 430 }},
{{ 189 , 249 },{ 340 , 460 }},
{{ 189 , 249 },{ 370 , 490 }},
{{ 189 , 249 },{ 400 , 520 }},
{{ 204 , 264 },{ 160 , 280 }},
{{ 204 , 264 },{ 190 , 310 }},
{{ 204 , 264 },{ 220 , 340 }},
{{ 204 , 264 },{ 250 , 370 }},
{{ 204 , 264 },{ 280 , 400 }},
{{ 204 , 264 },{ 310 , 430 }},
{{ 204 , 264 },{ 340 , 460 }},
{{ 204 , 264 },{ 370 , 490 }},
{{ 204 , 264 },{ 400 , 520 }},
{{ 219 , 279 },{ 160 , 280 }},
{{ 219 , 279 },{ 190 , 310 }},
{{ 219 , 279 },{ 220 , 340 }},
{{ 219 , 279 },{ 250 , 370 }},
{{ 219 , 279 },{ 280 , 400 }},
{{ 219 , 279 },{ 310 , 430 }},
{{ 219 , 279 },{ 340 , 460 }},
{{ 219 , 279 },{ 370 , 490 }},
{{ 219 , 279 },{ 400 , 520 }},
{{ 234 , 294 },{ 160 , 280 }},
{{ 234 , 294 },{ 190 , 310 }},
{{ 234 , 294 },{ 220 , 340 }},
{{ 234 , 294 },{ 250 , 370 }},
{{ 234 , 294 },{ 280 , 400 }},
{{ 234 , 294 },{ 310 , 430 }},
{{ 234 , 294 },{ 340 , 460 }},
{{ 234 , 294 },{ 370 , 490 }},
{{ 234 , 294 },{ 400 , 520 }},
{{ 249 , 309 },{ 160 , 280 }},
{{ 249 , 309 },{ 190 , 310 }},
{{ 249 , 309 },{ 220 , 340 }},
{{ 249 , 309 },{ 250 , 370 }},
{{ 249 , 309 },{ 280 , 400 }},
{{ 249 , 309 },{ 310 , 430 }},
{{ 249 , 309 },{ 340 , 460 }},
{{ 249 , 309 },{ 370 , 490 }},
{{ 249 , 309 },{ 400 , 520 }},
{{ 264 , 324 },{ 160 , 280 }},
{{ 264 , 324 },{ 190 , 310 }},
{{ 264 , 324 },{ 220 , 340 }},
{{ 264 , 324 },{ 250 , 370 }},
{{ 264 , 324 },{ 280 , 400 }},
{{ 264 , 324 },{ 310 , 430 }},
{{ 264 , 324 },{ 340 , 460 }},
{{ 264 , 324 },{ 370 , 490 }},
{{ 264 , 324 },{ 400 , 520 }},
{{ 279 , 339 },{ 160 , 280 }},
{{ 279 , 339 },{ 190 , 310 }},
{{ 279 , 339 },{ 220 , 340 }},
{{ 279 , 339 },{ 250 , 370 }},
{{ 279 , 339 },{ 280 , 400 }},
{{ 279 , 339 },{ 310 , 430 }},
{{ 279 , 339 },{ 340 , 460 }},
{{ 279 , 339 },{ 370 , 490 }},
{{ 279 , 339 },{ 400 , 520 }},
{{ 24 , 104 },{ 160 , 320 }},
{{ 24 , 104 },{ 200 , 360 }},
{{ 24 , 104 },{ 240 , 400 }},
{{ 24 , 104 },{ 280 , 440 }},
{{ 24 , 104 },{ 320 , 480 }},
{{ 24 , 104 },{ 360 , 520 }},
{{ 24 , 104 },{ 400 , 560 }},
{{ 44 , 124 },{ 160 , 320 }},
{{ 44 , 124 },{ 200 , 360 }},
{{ 44 , 124 },{ 240 , 400 }},
{{ 44 , 124 },{ 280 , 440 }},
{{ 44 , 124 },{ 320 , 480 }},
{{ 44 , 124 },{ 360 , 520 }},
{{ 44 , 124 },{ 400 , 560 }},
{{ 64 , 144 },{ 160 , 320 }},
{{ 64 , 144 },{ 200 , 360 }},
{{ 64 , 144 },{ 240 , 400 }},
{{ 64 , 144 },{ 280 , 440 }},
{{ 64 , 144 },{ 320 , 480 }},
{{ 64 , 144 },{ 360 , 520 }},
{{ 64 , 144 },{ 400 , 560 }},
{{ 84 , 164 },{ 160 , 320 }},
{{ 84 , 164 },{ 200 , 360 }},
{{ 84 , 164 },{ 240 , 400 }},
{{ 84 , 164 },{ 280 , 440 }},
{{ 84 , 164 },{ 320 , 480 }},
{{ 84 , 164 },{ 360 , 520 }},
{{ 84 , 164 },{ 400 , 560 }},
{{ 104 , 184 },{ 160 , 320 }},
{{ 104 , 184 },{ 200 , 360 }},
{{ 104 , 184 },{ 240 , 400 }},
{{ 104 , 184 },{ 280 , 440 }},
{{ 104 , 184 },{ 320 , 480 }},
{{ 104 , 184 },{ 360 , 520 }},
{{ 104 , 184 },{ 400 , 560 }},
{{ 124 , 204 },{ 160 , 320 }},
{{ 124 , 204 },{ 200 , 360 }},
{{ 124 , 204 },{ 240 , 400 }},
{{ 124 , 204 },{ 280 , 440 }},
{{ 124 , 204 },{ 320 , 480 }},
{{ 124 , 204 },{ 360 , 520 }},
{{ 124 , 204 },{ 400 , 560 }},
{{ 144 , 224 },{ 160 , 320 }},
{{ 144 , 224 },{ 200 , 360 }},
{{ 144 , 224 },{ 240 , 400 }},
{{ 144 , 224 },{ 280 , 440 }},
{{ 144 , 224 },{ 320 , 480 }},
{{ 144 , 224 },{ 360 , 520 }},
{{ 144 , 224 },{ 400 , 560 }},
{{ 164 , 244 },{ 160 , 320 }},
{{ 164 , 244 },{ 200 , 360 }},
{{ 164 , 244 },{ 240 , 400 }},
{{ 164 , 244 },{ 280 , 440 }},
{{ 164 , 244 },{ 320 , 480 }},
{{ 164 , 244 },{ 360 , 520 }},
{{ 164 , 244 },{ 400 , 560 }},
{{ 184 , 264 },{ 160 , 320 }},
{{ 184 , 264 },{ 200 , 360 }},
{{ 184 , 264 },{ 240 , 400 }},
{{ 184 , 264 },{ 280 , 440 }},
{{ 184 , 264 },{ 320 , 480 }},
{{ 184 , 264 },{ 360 , 520 }},
{{ 184 , 264 },{ 400 , 560 }},
{{ 204 , 284 },{ 160 , 320 }},
{{ 204 , 284 },{ 200 , 360 }},
{{ 204 , 284 },{ 240 , 400 }},
{{ 204 , 284 },{ 280 , 440 }},
{{ 204 , 284 },{ 320 , 480 }},
{{ 204 , 284 },{ 360 , 520 }},
{{ 204 , 284 },{ 400 , 560 }},
{{ 224 , 304 },{ 160 , 320 }},
{{ 224 , 304 },{ 200 , 360 }},
{{ 224 , 304 },{ 240 , 400 }},
{{ 224 , 304 },{ 280 , 440 }},
{{ 224 , 304 },{ 320 , 480 }},
{{ 224 , 304 },{ 360 , 520 }},
{{ 224 , 304 },{ 400 , 560 }},
{{ 244 , 324 },{ 160 , 320 }},
{{ 244 , 324 },{ 200 , 360 }},
{{ 244 , 324 },{ 240 , 400 }},
{{ 244 , 324 },{ 280 , 440 }},
{{ 244 , 324 },{ 320 , 480 }},
{{ 244 , 324 },{ 360 , 520 }},
{{ 244 , 324 },{ 400 , 560 }},
{{ 264 , 344 },{ 160 , 320 }},
{{ 264 , 344 },{ 200 , 360 }},
{{ 264 , 344 },{ 240 , 400 }},
{{ 264 , 344 },{ 280 , 440 }},
{{ 264 , 344 },{ 320 , 480 }},
{{ 264 , 344 },{ 360 , 520 }},
{{ 264 , 344 },{ 400 , 560 }},
{{ 24 , 124 },{ 160 , 360 }},
{{ 24 , 124 },{ 210 , 410 }},
{{ 24 , 124 },{ 260 , 460 }},
{{ 24 , 124 },{ 310 , 510 }},
{{ 24 , 124 },{ 360 , 560 }},
{{ 74 , 174 },{ 160 , 360 }},
{{ 74 , 174 },{ 210 , 410 }},
{{ 74 , 174 },{ 260 , 460 }},
{{ 74 , 174 },{ 310 , 510 }},
{{ 74 , 174 },{ 360 , 560 }},
{{ 124 , 224 },{ 160 , 360 }},
{{ 124 , 224 },{ 210 , 410 }},
{{ 124 , 224 },{ 260 , 460 }},
{{ 124 , 224 },{ 310 , 510 }},
{{ 124 , 224 },{ 360 , 560 }},
{{ 174 , 274 },{ 160 , 360 }},
{{ 174 , 274 },{ 210 , 410 }},
{{ 174 , 274 },{ 260 , 460 }},
{{ 174 , 274 },{ 310 , 510 }},
{{ 174 , 274 },{ 360 , 560 }},
};
