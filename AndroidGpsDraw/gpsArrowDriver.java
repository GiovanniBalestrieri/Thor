package com.example.mrdeeppurple.gpsarrowdrawer;

import android.content.Context;
import android.content.res.Resources;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Matrix;
import android.graphics.drawable.BitmapDrawable;
import android.graphics.drawable.Drawable;
import android.util.AttributeSet;
import android.util.Log;
import android.widget.ImageView;

/**
 * Created by mrdeeppurple on 12/04/15.
 */
public class gpsArrowDriver extends ImageView {
    private Canvas cnv;
    private Bitmap bmp;
    private double [] xTargetArray;
    private double [] yTargetArray;
    private int canvasSizeX;
    private int canvasSizeY;
    private double gpsPosX;
    private double gpsPosY;
    private Bitmap arrowBmp;
    private int arrowPoseX;
    private int arrowPoseY;
    private int pointindex;


    public gpsArrowDriver(Context context) {
        super(context);
        xTargetArray = null;
        yTargetArray = null;
        bmp = null;
        gpsPosX = 0;
        gpsPosY = 0;
        Resources res = this.getResources();
        Drawable arrow = res.getDrawable(R.drawable.arrow);
        if(arrow != null) {
            arrowBmp = drawableToBitmap(arrow);
        }
        pointindex = -1;
    }
    public gpsArrowDriver(Context context, AttributeSet attrs){
        super(context, attrs);
        xTargetArray = null;
        yTargetArray = null;
        bmp = null;
        gpsPosX = 0;
        gpsPosY = 0;
        Resources res = this.getResources();
        Drawable arrow = res.getDrawable(R.drawable.arrow);
        if(arrow != null) {
            arrowBmp = drawableToBitmap(arrow);
        }
        pointindex = -1;
    }
    public gpsArrowDriver(Context context, AttributeSet attrs, int defStyle){
        super(context, attrs, defStyle);
        xTargetArray = null;
        yTargetArray = null;
        bmp = null;
        gpsPosX = 0;
        gpsPosY = 0;
        Resources res = this.getResources();
        Drawable arrow = res.getDrawable(R.drawable.arrow);
        if(arrow != null) {
            arrowBmp = drawableToBitmap(arrow);
        }
        pointindex = -1;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        cnv = canvas;
        if(bmp == null) {
            Drawable image = getDrawable();
            if (image == null || image.getIntrinsicHeight() == 0 || image.getIntrinsicWidth() == 0) {
                // Se non è specificata nessuna immagine sul canvas, disegna il background bianco
                cnv.drawRGB(255, 255, 255);
            } else {
                bmp = drawableToBitmap(image);
                cleanDraw();
            }
        }
        else{
            cleanDraw();
        }
        canvasSizeX = canvas.getWidth();
        canvasSizeY = canvas.getHeight();
        arrowPoseX = canvasSizeX/2-arrowBmp.getWidth()/2;
        arrowPoseY = canvasSizeY-arrowBmp.getHeight();
        if(xTargetArray == null || yTargetArray == null){
            drawArrow(0.0, 0.0, 0.0, 0.0);
        }
        else {
            if(pointindex == -1) {
                /* il primo punto che prendo è il più vicino alla mia posizione attuale */
                pointindex = findClosestPoint(gpsPosX, gpsPosY, xTargetArray, yTargetArray);
            }
            if (pointindex >= 0 && pointindex < xTargetArray.length) {
                drawArrow(gpsPosX, gpsPosY, xTargetArray[pointindex], yTargetArray[pointindex]);
                if(Math.sqrt(Math.pow(gpsPosX-xTargetArray[pointindex], 2)+ Math.pow(gpsPosY - yTargetArray[pointindex], 2)) < 0.8){
                    /*80 cm di precisione */
                    /* se sono più vicino di 80cm dal punto, cerco il punto successivo */
                    pointindex += 1;
                }
            }
        }
    }

    private Bitmap drawableToBitmap(Drawable drawable)
    {
        if (drawable == null) {
            return null;
        }
        else if (drawable instanceof BitmapDrawable) {
            int bitmapSizeX = drawable.getIntrinsicWidth();
            int bitmapSizeY = drawable.getIntrinsicHeight();
            return Bitmap.createBitmap(((BitmapDrawable) drawable).getBitmap(), 0, 0, bitmapSizeX, bitmapSizeY);
        }

        Bitmap bitmap = Bitmap.createBitmap(drawable.getIntrinsicWidth(), drawable.getIntrinsicHeight(), Bitmap.Config.ARGB_8888);
        Canvas canvas = new Canvas(bitmap);
        drawable.setBounds(0, 0, canvas.getWidth(), canvas.getHeight());
        drawable.draw(canvas);

        return bitmap;
    }

    /**********************************************************************************************
     *
     * FUNZINI DA UTILIZZARE
     * updatePath() imposta il path come due array di double
     * updateActualPosition() prende come parametri double la posizione del gps e aggiorna la freccia
     * loadMap() carica una bitmap da usare come sfondo
     */

    public void updatePath(double patharrayx[], double patharrayy[]){
        xTargetArray = patharrayx;
        yTargetArray = patharrayy;
    }

    public void updateActualPosition(double x, double y){
        gpsPosX = x;
        gpsPosY = y;
        invalidate();
    }

    public void loadMap(Bitmap b){
        bmp = b;
    }
    private void cleanDraw(){
        if(bmp != null)
            cnv.drawBitmap(bmp, bmp.getHeight(), bmp.getWidth(), null);
        else
            cnv.drawRGB(255, 255, 255);
    }

    private void drawArrow(double actx, double acty, double destx, double desty){
        if(cnv != null && arrowBmp != null) {
            float deg = 0;
            double div = destx - actx;
            if (div != 0) {
                deg = ((float) Math.atan2(desty - acty, div));
                deg = deg*((float)57.2957795);
            }
            Matrix matrix = new Matrix();
            matrix.postRotate(deg);
            Bitmap rArrowBmp = Bitmap.createBitmap(arrowBmp, 0, 0, arrowBmp.getWidth(), arrowBmp.getHeight(), matrix, true);
            cnv.drawBitmap(rArrowBmp, arrowPoseX, arrowPoseY, null);
        }
    }

    private int findClosestPoint(double x, double y, double xarray[], double yarray[]){
        int lx = xarray.length;
        int ly = yarray.length;
        double distance[];

        if(lx == ly){
            distance = new double[lx];
            for(int i = 0; i< lx; i++){
                distance[i] = Math.sqrt(Math.pow(xarray[i]-x, 2)+Math.pow(yarray[i]-y, 2));
            }
            return minIndex(distance);
        }
        return -1;
    }

    private int minIndex(double a[]){
        if(a != null){
            int l = a.length;
            if(l > 0){
                double lastMinVal = a[0];
                int min = 0;
                for(int i = 0; i<l; i++){
                    if(a[i]<lastMinVal){
                        lastMinVal = a[i];
                        min = i;
                    }
                }
                return min;
            }
            return -1;
        }
        return -1;
    }
}