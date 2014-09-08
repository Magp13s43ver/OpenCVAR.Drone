/**
 * @file display_stage.c
 * @author nicolas.brulez@parrot.com
 * @date 2012/09/25
 *
 * This stage is a naive example of how to display video using GTK2 + Cairo
 * In a complete application, all GTK handling (gtk main thread + widgets/window creation)
 *  should NOT be handled by the video pipeline (see the Navigation linux example)
 *
 * The window will be resized according to the picture size, and should not be resized bu the user
 *  as we do not handle any gtk event except the expose-event
 *
 * This example is not intended to be a GTK/Cairo tutorial, it is only an example of how to display
 *  the AR.Drone live video feed. The GTK Thread is started here to improve the example readability
 *  (we have all the gtk-related code in one file)
 */

// Self header file
#include "display_stage.h"

//OpenCV
#include </home/brandonteh/Desktop/OpenCV/opencv-2.4.9/include/opencv/cv.h>
#include </usr/include/opencv/highgui.h>
#include </home/brandonteh/Desktop/OpenCV/opencv-2.4.9/modules/ml/include/opencv2/ml/ml.hpp>

// GTK/Cairo headers
#include <cairo.h>
#include <gtk/gtk.h>

// Funcs pointer definition
const vp_api_stage_funcs_t display_stage_funcs = {
    NULL,
    (vp_api_stage_open_t) display_stage_open,
    (vp_api_stage_transform_t) display_stage_transform,
    (vp_api_stage_close_t) display_stage_close
};

static int temp_x, temp_y, cur_x, cur_y, count;
/*
int get_location(int *x, int *y)
{
    *x = face_x;
    *y = face_y; 
    return 1;
}
*/
void doMosaic(IplImage* in, int x0, int y0, int width, int height, int size)
{
	int b, g, r, col, row;

  	int xMin = size*(int)floor((double)x0/size);
  	int yMin = size*(int)floor((double)y0/size);
  	int xMax = size*(int)ceil((double)(x0+width)/size);
  	int yMax = size*(int)ceil((double)(y0+height)/size);
  
  	printf("rect: x=%d y=%d w=%d h=%d s=%d\n", x0, y0, width, height, size);
  	
	int y, x, i, j;
	for(y=yMin; y<yMax; y+=size)
	{
    		for(x=xMin; x<xMax; x+=size)
		{
      			b = g = r = 0;

      			for(i=0; i<size; i++)
			{
        			if( y+i > in->height )
				{
          				break;
       				}
	
        			row = i;

        			for(j=0; j<size; j++)
				{
          				if( x+j > in->width )
					{
			        		break;
          				}

          			b += (unsigned char)in->imageData[in->widthStep*(y+i)+(x+j)*3];
          			g += (unsigned char)in->imageData[in->widthStep*(y+i)+(x+j)*3+1];
          			r += (unsigned char)in->imageData[in->widthStep*(y+i)+(x+j)*3+2];
          			col = j;
        			}
      			}

      			row++;
      			col++;

      			for(i=0;i<row;i++)
			{
 		        	for(j=0;j<col;j++)
				{
          				in->imageData[in->widthStep*(y+i)+(x+j)*3]   = cvRound((double)b/(row*col));
          				in->imageData[in->widthStep*(y+i)+(x+j)*3+1] = cvRound((double)g/(row*col));
          				in->imageData[in->widthStep*(y+i)+(x+j)*3+2] = cvRound((double)r/(row*col));
        			}
      			}
    		}
 	}
}

IplImage *ipl_image_from_data(uint8_t* data, int reduced_image, int width, int height)
{
  	IplImage *currframe;
  	IplImage *dst;

  	currframe = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 3);
  	dst = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 3);

  	currframe->imageData = data;
  	cvCvtColor(currframe, dst, CV_BGR2RGB);
  	cvReleaseImage(&currframe);

  	return dst;
}

// Extern so we can make the ardrone_tool_exit() function (ardrone_testing_tool.c)
// return TRUE when we close the video window
extern int exit_program;

// Boolean to avoid asking redraw of a not yet created / destroyed window
bool_t gtkRunning = FALSE;

// Picture size getter from input buffer size
// This function only works for RGB565 buffers (i.e. 2 bytes per pixel)
static void getPicSizeFromBufferSize (uint32_t bufSize, uint32_t *width, uint32_t *height)
{
    if (NULL == width || NULL == height)
    {
        return;
    }

    switch (bufSize)
    {
    case 50688: //QCIF > 176*144 *2bpp
        *width = 176;
        *height = 144;
        break;
    case 153600: //QVGA > 320*240 *2bpp
        *width = 320;
        *height = 240;
        break;
    case 460800: //360p > 640*360 *2bpp
        *width = 640;
        *height = 360;
        break;
    case 1843200: //720p > 1280*720 *2bpp
        *width = 1280;
        *height = 720;
        break;
    default:
        *width = 0;
        *height = 0;
        break;
    }
}

// Get actual frame size (without padding)
void getActualFrameSize (display_stage_cfg_t *cfg, uint32_t *width, uint32_t *height)
{
    if (NULL == cfg || NULL == width || NULL == height)
    {
        return;
    }

    *width = cfg->decoder_info->width;
    *height = cfg->decoder_info->height;
}

// Redraw function, called by GTK each time we ask for a frame redraw
static gboolean
on_expose_event (GtkWidget *widget,
                 GdkEventExpose *event,
                 gpointer data)
{
    display_stage_cfg_t *cfg = (display_stage_cfg_t *)data;

    if (2.0 != cfg->bpp)
    {
        return FALSE;
    }

    uint32_t width = 0, height = 0, stride = 0;
    getPicSizeFromBufferSize (cfg->fbSize, &width, &height);
    stride = cfg->bpp * width;

    if (0 == stride)
    {
        return FALSE;
    }

    uint32_t actual_width = 0, actual_height = 0;
    getActualFrameSize (cfg, &actual_width, &actual_height);
    gtk_window_resize (GTK_WINDOW (widget), actual_width, actual_height);

    cairo_t *cr = gdk_cairo_create (widget->window);

    cairo_surface_t *surface = cairo_image_surface_create_for_data (cfg->frameBuffer, CAIRO_FORMAT_RGB16_565, width, height, stride);

    cairo_set_source_surface (cr, surface, 0.0, 0.0);

    cairo_paint (cr);

    cairo_surface_destroy (surface);

    cairo_destroy (cr);

    return FALSE;
}

/**
 * Main GTK Thread.
 * On an actual application, this thread should be started from your app main thread, and not from a video stage
 * This thread will handle all GTK-related functions
 */
DEFINE_THREAD_ROUTINE(gtk, data)
{
    GtkWidget *window = gtk_window_new (GTK_WINDOW_TOPLEVEL);

    display_stage_cfg_t *cfg = (display_stage_cfg_t *)data;
    cfg->widget = window;

    g_signal_connect (window, "expose-event", G_CALLBACK (on_expose_event), data);
    g_signal_connect (window, "destroy", G_CALLBACK (gtk_main_quit), NULL);

    gtk_window_set_position (GTK_WINDOW (window), GTK_WIN_POS_CENTER);
    gtk_window_set_default_size (GTK_WINDOW (window), 10, 10);
    gtk_widget_set_app_paintable (window, TRUE);
    gtk_widget_set_double_buffered (window, FALSE);

    gtk_widget_show_all (window);

    gtkRunning = TRUE;

    gtk_main ();

    gtkRunning = FALSE;

    // Force ardrone_tool to close
    exit_program = 0;

    // Sometimes, ardrone_tool might not finish properly
    // This happens mainly because a thread is blocked on a syscall
    // in this case, wait 5 seconds then kill the app
    sleep (5);
    exit (0);

    return (THREAD_RET)0;
}

C_RESULT display_stage_open (display_stage_cfg_t *cfg)
{
    // Check that we use RGB565
    if (2 != cfg->bpp)
    {
        // If that's not the case, then don't display anything
        cfg->paramsOK = FALSE;
    }
    else
    {
        // Else, start GTK thread and window
        cfg->paramsOK = TRUE;
        cfg->frameBuffer = NULL;
        cfg->fbSize = 0;
        START_THREAD (gtk, cfg);
    }
    return C_OK;
}

C_RESULT display_stage_transform (display_stage_cfg_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
	if(count > 1)
	{
		temp_x = cur_x;
		temp_y = cur_y;
	}

	int i = 0;
  	const char *cascade_name = "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml";
  	CvHaarClassifierCascade *cascade = 0;
  	CvMemStorage *storage = 0;
  	CvSeq *faces;

	cascade = (CvHaarClassifierCascade *) cvLoad (cascade_name, 0, 0, 0);

	if(!cascade)
	{
		printf("Could not load cascade.\n");
		return -1;
	}
  	
	uint32_t width = 0, height = 0;
    	getPicSizeFromBufferSize (in->size, &width, &height);

	//Obtain image from ARDrone and convert it to OpenCV format.
    	IplImage *img = ipl_image_from_data((uint8_t*)in->buffers[0], 1, 640, 360);

	storage = cvCreateMemStorage (0);
    	cvClearMemStorage (storage);

	//DetectFaces
	faces = cvHaarDetectObjects (img, cascade, storage, 1.11, 4, 0, cvSize(30, 30), cvSize (0, 0));
    	

	for (i = 0; i < (faces ? faces->total : 0); i++) 
	{
		CvRect *r = (CvRect *) cvGetSeqElem (faces, i);
      		doMosaic(img, r->x, r->y, r->width, r->height, 10);
		cur_x = r->x;
		cur_y = r->y;
		//printf("In the loop.\n");
		count++; //increases count when detect faces
 	}
	
	if(count > 2)
	{
		printf("prev_x = %d, prev_y = %d, cur_x = %d, cur_y = %d\n", temp_x, temp_y, cur_x, cur_y);
		if(cur_x - temp_x > 3)
		{
			//printf("Move Right.\n");
			ardrone_at_set_progress_cmd( 1, 1.0, 0.0, 0.0, 0.0 );
		}

		if(cur_x - temp_x < -3)
		{
			//printf("Move Left.\n");
			ardrone_at_set_progress_cmd( 1, -1.0, 0.0, 0.0, 0.0 );
		}

		if(cur_y - temp_y > 3)
		{
			//printf("Move Down.\n");
		        ardrone_at_set_progress_cmd( 1, 0.0, 0.0, -1.0, 0.0 );
		}

		if(cur_y - temp_y < -5)
		{
			//printf("Move Up.\n");
			//ardrone_at_reset_com_watchdog();
		        ardrone_tool_set_progressive_cmd( 1, 0.0, 0.0, 0.3, 0.0 );
			//vp_os_delay(1000);
		}
	}

	//emergency++;p

	if(count > 50)
	{
		printf("Exit\n");
		ardrone_tool_set_ui_pad_start(0);
		return -1;
	}

	cvNamedWindow("FaceDetect", CV_WINDOW_AUTOSIZE);
	cvShowImage("FaceDetect", img);
	cvWaitKey(1);
	cvReleaseImage(&img);

    return C_OK;
}

C_RESULT display_stage_close (display_stage_cfg_t *cfg)
{
    // Free all allocated memory
    if (NULL != cfg->frameBuffer)
    {
        vp_os_free (cfg->frameBuffer);
        cfg->frameBuffer = NULL;
    }

    return C_OK;
}
