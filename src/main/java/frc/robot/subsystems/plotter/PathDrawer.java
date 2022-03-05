package frc.robot.subsystems.plotter;

import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;

import java.awt.*;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.Scanner;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;



//Plot points from positionEstimator
//Goal: Plot vectors using the angle as well as points from position Estimator

public class PathDrawer extends JFrame {
    private ArrayList<DataPoint> dataSet;
    private GraphPanel graphPanel = null;
    private File lastFolderName = null; 
    private File lastFile = null;
    // Data file extension - used for reading and writing
    private final String DATA_FILE_EXTENSION = "txt";


    public PathDrawer(int w, int h){
        setTitle("Data Graph");
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setPreferredSize(new Dimension(w, h + 24));
        setFocusable(true);
        lastFolderName = new File(System.getProperty("user.dir"));

        graphPanel = new GraphPanel();
        graphPanel.grabFocus();

        setJMenuBar(makeMenuBar()); 
        
        setupMouse();

        add(graphPanel);
        pack();
        setVisible(true);

        dataSet = readFile(new File("28-01-2022-20-59-13.txt"));
        updateGraphPanel();
        setSize(w, h);      
    }

    public void updateGraphPanel() {
        graphPanel.setDataPoints(dataSet);
        graphPanel.setRadius(2);
    }
    public static void main(String[] args) {
        EventQueue.invokeLater(() -> {
            PathDrawer pathDrawer = new PathDrawer(800, 600);
            pathDrawer.setVisible(true);
        });
    }
  
    public ArrayList<DataPoint> readFile(File file) {
        
        ArrayList<DataPoint> dataPoints = new ArrayList<>();

        try {
            // File myObj = new File("28-01-2022-20-59-13.txt");
            Scanner myReader = new Scanner(file);
            while (myReader.hasNextLine()) {
                String data = myReader.nextLine();
                //List<String> dataPoint = Arrays.asList(data.split(","));
                String[] dataPointStr = data.split(",");
                long timeStamp = Long.parseLong(dataPointStr[0]);
                double xPos = Double.parseDouble(dataPointStr[1]);
                double yPos = Double.parseDouble(dataPointStr[2]);
                double angle = Double.parseDouble(dataPointStr[3]);
                double actRot = Double.parseDouble(dataPointStr[4]);
                double xS = Double.parseDouble(dataPointStr[5]);
                double yS = Double.parseDouble(dataPointStr[6]);
                

                DataPoint dataPoint = new DataPoint(timeStamp, xPos, yPos, angle, actRot, xS, yS);
                dataPoints.add(dataPoint);
            }
            myReader.close();
        } catch (FileNotFoundException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }
        
        return dataPoints;

    }

    public void setupMouse(){
      addMouseListener(new MouseListener() {
          @Override
          public void mousePressed(MouseEvent e) {
              // may need to add getters of some sort for xPos & yPos in GraphPanel
              // in order for this position comparison to work
              int x = e.getX();
              int y = e.getY();
              Iterator<DataPoint> iter = dataSet.iterator();
              DataPoint dataPoint = iter.next();
                  while(iter.hasNext()) {
                    if(x < dataPoint.x*100-5 && x > dataPoint.x*100+5 && y < dataPoint.y*100+5) {

                    }
                  }
          }
          @Override
          public void mouseReleased(MouseEvent e) { }
          @Override
          public void mouseClicked(MouseEvent e) { }
          @Override
          public void mouseEntered(MouseEvent e) { }
          @Override
          public void mouseExited(MouseEvent e) { }
      });
    }

    private JMenuBar makeMenuBar()
    {
        // the menu bar is the base container for menus
        var menuBar = new JMenuBar();

        // Each menu is one 'column' of commands inside of the menu bar.
        // the first menu to go in the menu bar is the File menu
        // to close the application and to choose a file to open
        var menu = new JMenu("File");
        // add keyboard shortcut - Alt+F (or Cmd+F)
        menu.setMnemonic(KeyEvent.VK_F);

        // each menu is a collection of menu items
        JMenuItem menuItem = new JMenuItem("Open File...");
        menuItem.setMnemonic(KeyEvent.VK_O);
        // add code to call a method to handle the event when
        // a user clicks the menu item
        menuItem.addActionListener((event) -> selectFile());
        menu.add(menuItem);

        menuItem = new JMenuItem("Save As...");
        menuItem.setMnemonic(KeyEvent.VK_S);
        menuItem.addActionListener((event) -> saveFile());
        menu.add(menuItem);

        menuItem = new JMenuItem("Quit");
        menuItem.setMnemonic(KeyEvent.VK_Q);
        menuItem.addActionListener((event) -> System.exit(0));
        menu.add(menuItem);

        // Now add the menu to the menu bar
        menuBar.add(menu);

        return menuBar;

    }

    /**
     * Opens a file selection window and once the user selects a file, updates the label on the screen to show the filename. 
     */
    private void selectFile()
    {
        File selectedFile = null; 
        System.out.println("Current folder " + lastFolderName.getName());
        // initialize the file selection window
        final JFileChooser fc = new JFileChooser();
        fc.setFileFilter(new FileNameExtensionFilter("Text Files", DATA_FILE_EXTENSION));
        fc.setCurrentDirectory(lastFolderName);
        // Shows the window and waits to see if the user selected a file or clicked cancel
        int returnVal = fc.showOpenDialog(this);

        // Check if the user selected a file
        if(returnVal == JFileChooser.APPROVE_OPTION) {
            // get a handle on the file the user selected
            selectedFile = fc.getSelectedFile();
            lastFolderName = fc.getCurrentDirectory();
            lastFile = selectedFile;
            // System.out.println("hello");
            dataSet = readFile(selectedFile);
    
            updateGraphPanel();
            repaint();
            System.out.println("opening file " + selectedFile.getPath());
        }
        else {
            System.out.println("No file selected");
        }
    }

    private void saveFile() {
        // initialize the file selection window
        final JFileChooser fc = new JFileChooser();
        fc.setCurrentDirectory(lastFolderName);
        // Shows the window and waits to see if the user selected a file or clicked cancel
        int returnVal = fc.showSaveDialog(this);

        if(returnVal == JFileChooser.APPROVE_OPTION) {
            File saveAsFile = fc.getSelectedFile();
            System.out.println("File to save as: " + saveAsFile.getPath() +".txt");
            Path sourcePath = FileSystems.getDefault().getPath(lastFile.getPath());
            Path destinationPath = FileSystems.getDefault().getPath(
                saveAsFile.getPath() + "." + DATA_FILE_EXTENSION);
            try {
                Files.copy(sourcePath, destinationPath, StandardCopyOption.REPLACE_EXISTING);
            }
            catch (IOException ioe) {
                System.err.println("Error: Unable to save file");
            }
        }
    }
}
