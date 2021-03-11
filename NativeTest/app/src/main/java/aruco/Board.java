package aruco;

import org.opencv.core.*;

import java.util.Vector;

/**
 * Class representing a board of markers. Each board will be identified
 * by its configuration. A 3d model can be attached to a board and by default
 * will be rendered on its center. A 3d axis can be drawn as well using OpenCV
 * functions.
 *
 * @author Rafael Ortega
 */
public class Board extends Vector<Marker> {
    private static final long serialVersionUID = 1L;

    // fields
    protected BoardConfiguration conf;
    protected Mat Rvec, Tvec;
    protected float markerSizeMeters;

    // constructor
    public Board() {
        Rvec = new Mat(3, 1, CvType.CV_64FC1);
        Tvec = new Mat(3, 1, CvType.CV_64FC1);
        markerSizeMeters = -1;
    }

    // other methods

    public Mat createBoardImage(Size gridSize, int markerSize, int markerDistance, int firstID, BoardConfiguration info) throws CvException {
        info.markersId = new int[(int) gridSize.width][(int) gridSize.height];
        int nMarkers = (int) (gridSize.width * gridSize.height);
        if (firstID + nMarkers >= 1024)
            throw new CvException("creation of board implies a marker with an incorrect ID");

        int idp = 0;
        int[] data = new int[info.width * info.height];
        for (int i = 0; i < gridSize.height; i++)
            for (int j = 0; j < gridSize.width; j++, idp++)
                info.markersId[i][j] = firstID + idp; //number in the range [0,1023]
        info.markerSizePix = markerSize;
        info.markerDistancePix = markerDistance;

        int sizeY = (int) gridSize.height * markerSize + ((int) gridSize.height - 1) * markerDistance;
        int sizeX = (int) gridSize.width * markerSize + ((int) gridSize.width - 1) * markerDistance;
        Mat tableImage = new Mat(sizeY, sizeX, CvType.CV_8UC1);
        tableImage.setTo(new Scalar(255));

        for (int y = 0; y < gridSize.height; y++)
            for (int x = 0; x < gridSize.width; x++) {
                Mat subrect = tableImage.submat(x * (markerDistance + markerSize), x * (markerDistance + markerSize) + markerSize,
                        y * (markerDistance + markerSize), y * (markerDistance + markerSize) + markerSize);
                Mat marker = Marker.createMarkerImage(data[y * ((int) gridSize.height) + x], markerSize);
                marker.copyTo(subrect);
            }

        return tableImage;
    }

    public void draw3dAxis(Mat frame, CameraParameters cp, Scalar color) {
        Utils.draw3dAxis(frame, cp, color, 2 * this.get(0).ssize, Rvec, Tvec);
    }
}
