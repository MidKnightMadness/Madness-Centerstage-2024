package org.firstinspires.ftc.teamcode.Camera;

import org.opencv.core.Rect;

public class RectangleFactory {

    static Rect generateRectFrom2Points(int x1, int y1, int x2, int y2) {
        int width = x2 - x1;
        int height = y2 - y1;

        return new Rect(x1, y1, width, height);
    }

    static Rect generateRectFromPercentages(int width, int height, int xPercent1, int yPercent1, int xPercent2, int yPercent2) {
        int x1 = (int) (width * xPercent1 / 100d);
        int x2 = (int) (width * xPercent2 / 100d);
        int y1 = (int) (height * yPercent1 / 100d);
        int y2 = (int) (height * yPercent2 / 100d);

        return generateRectFrom2Points(x1, y1, x2, y2);
    }

}
