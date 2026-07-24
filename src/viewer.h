#pragma once

int  viewer_init(int width, int height, const char *title);
void viewer_free(void);
int  viewer_should_close(void);
int  viewer_is_paused(void);
void viewer_render(void);
