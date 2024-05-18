package team696.frc.lib.Log;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOError;
import java.lang.reflect.Method;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Date;
import java.util.Deque;
import java.util.List;
import java.util.TimeZone;
import java.util.concurrent.Callable;
import java.util.regex.Matcher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import team696.frc.robot.Robot;
import team696.frc.robot.util.Util;

public class Logger {
    public static Logger m_Logger;
    private Thread main;

    public enum type {
        none(0), 
        minimal(1), 
        debug(2);

        private int m_severity;
        private type(int severity) {
            m_severity = severity;
        }

        public boolean worse(type other) {
            return this.m_severity <= other.m_severity;
        }
    }

    private static class Loggable {
        public type Type;
        public Callable<Object> func;
        public String name;
        public StringPublisher ntPublisher;

        public Loggable(type t, String n, Callable<Object> f) {
            Type = t;
            func = f;
            name = n;
            ntPublisher = NetworkTableInstance.getDefault().getTable("*** Logger ***").getStringTopic(name).publish();
        }
    }

    private static final String[] m_DriveRoots = { "/media/sdb1/", "/media/sda1/", "/media/sdc1/" };

    private static final DateFormat dateFormat = new SimpleDateFormat("yy-MMM-dd-hh-mm-ss-SS-aa");
    private static final DateFormat simpleDateFormat = new SimpleDateFormat("mm:ss.SSS");

    //private HashMap<String, Callable<Object>> m_ToLog = new HashMap<String, Callable<Object>>();
    private static List<Loggable> m_ToLog = new ArrayList<Loggable>();

    private static Deque<String> m_ToWrite = new ArrayDeque<String>();
    private Deque<String> m_ToWriteCSV = new ArrayDeque<String>();

    private type m_logType = type.none;

    private String curDirectory = "";


    private boolean m_disabled = false;
    
    public static void log(String Name, String Value) {
        m_ToWrite.add(getSimpleCurrentTimeFormatted() + " -> [" + Name + "] " + Value + "\n");
        if (m_ToWrite.size() > 400) m_ToWrite.clear();
    }

    private Logger(type LogType) {
        m_logType = LogType;
        dateFormat.setTimeZone(TimeZone.getTimeZone("PST"));
        main = new Thread(new LogThread());
        main.setDaemon(true);
    }

    public static void addClassToLog(Object... objectsToAdd) {
        for (Object objectToAdd : objectsToAdd) {
            for (Method method : objectToAdd.getClass().getDeclaredMethods()) {
                if (method.isAnnotationPresent(Log.class))
                    m_ToLog.add(new Loggable(type.minimal, objectToAdd.getClass().getSimpleName() + "/" + method.getName(), ()->method.invoke(objectToAdd)));    
                
                if (method.isAnnotationPresent(Debug.class))
                    m_ToLog.add(new Loggable(type.debug  , objectToAdd.getClass().getSimpleName() + "/" + method.getName(), ()->method.invoke(objectToAdd)));    
            }
        }

        if (m_Logger != null && m_Logger.curDirectory != "")
            m_Logger.updateHeaders(m_Logger.curDirectory);
    }

    public static void registerLoggable(type logType, String name, Callable<Object> func) {
        m_ToLog.add(new Loggable(logType, name, func));    

        if (m_Logger != null && m_Logger.curDirectory != "")
            m_Logger.updateHeaders(m_Logger.curDirectory);
    } 

    public static Logger init(type LogType, Object... objects) {
        if (m_Logger != null) {
            PLog.info("Logger", "Don't Reinitialize The Logger");
            return m_Logger;
        }

        m_Logger = new Logger(LogType);
        addClassToLog(objects);
        return m_Logger;
    }

    public void start() {
        if (main.isAlive()) {
            PLog.info("Logger", "Don't Start The Logger Twice");
            return;
        }
        m_disabled = false;
        main.start();
    }

    public static Logger get() {
        return m_Logger;
    }

    private class LogThread implements Runnable {

        @Override
        public void run() {
            FileWriter writer;
            FileWriter writerCSV;
            String headers = "time,";
            try {
                curDirectory = getDirectory();
                File logDir = new File(curDirectory);
                logDir.mkdir();
            } catch (Exception e) {
                PLog.fatalException("Logger", "Failed To Write File", e);
                return;
            }

            PLog.info("Logger","Started Logging at " + curDirectory);

            for (Loggable func : m_ToLog) {
                if (!func.Type.worse(m_logType)) continue;
                headers += func.name + ",";
            }
            try {
                writerCSV = new FileWriter(curDirectory + "/values.csv", true);
                writerCSV.write(headers + "\n");
                writerCSV.close();
            } catch (Exception e) {
                PLog.fatalException("Logger", "Failed to write headers", e);
            }
            while (true) {
                if (m_disabled) continue;

                String time = getSimpleCurrentTimeFormatted();
                m_ToWriteCSV.add(time);
                for (Loggable loggable : m_ToLog) {
                    if (!loggable.Type.worse(m_logType)) continue;
                    try {
                        Object key = (loggable.func.call());
                        String Key = key.toString();
                        loggable.ntPublisher.set(Key);
                        m_ToWriteCSV.add("\"" + Key + "\"");
                    } catch (Exception e) {
                        m_ToWriteCSV.add("");
                    }
                }
     
                try {
                    writerCSV = new FileWriter(curDirectory + "/values.csv", true);
                    writer = new FileWriter(curDirectory + "/main.log", true);
                    while (m_ToWrite.size() > 0) {
                        writer.write(m_ToWrite.pop());
                    }          
                    while (m_ToWriteCSV.size() > 0) {
                        writerCSV.write(m_ToWriteCSV.pop() + ",");
                    }          
                    writerCSV.write("\n");
                    writer.close();
                    writerCSV.close();
                }
                catch (Exception e) {
                    PLog.fatalException("Logger", "Failed to log value", e);
                }
                Util.sleep(150);
            }
        }
    }

    public static void close() {
        if (m_Logger == null) return;

        m_Logger.m_disabled = true;

        try {
            m_Logger.main.join(300);
        } catch(Exception e) {
            PLog.fatalException("Logger", "Failed to close logging thread", e);
        }

        FileWriter writer;
        FileWriter writerCSV;

        try {
            writerCSV = new FileWriter(m_Logger.curDirectory + "/values.csv", true);
            writer = new FileWriter(m_Logger.curDirectory + "/main.log", true);
            while (m_ToWrite.size() > 0) {
                writer.write(m_ToWrite.pop());
            }          
            while (m_Logger.m_ToWriteCSV.size() > 0) {
                writerCSV.write(m_Logger.m_ToWriteCSV.pop() + ",");
            }          
            writerCSV.write("\n");
            writer.close();
            writerCSV.close();
        }
        catch (Exception e) {
            PLog.fatalException("Logger", "Failed to log value", e);
        }
    }

    private void updateHeaders(String dir) {
        try{
            String headers = "time,";
            for (Loggable func : m_ToLog) {
                if (!func.Type.worse(m_logType)) continue;
                headers += func.name + ",";
            }

            File file = new File(curDirectory + "/values.csv");
            BufferedReader reader = new BufferedReader(new FileReader(file));

            String words = "", list = "", first = null;
            while ((words = reader.readLine()) != null) {
                if (first == null) first = words;
                list += words + "\r\n";
            }
            reader.close();
            String replacedtext = list.replaceAll(first, Matcher.quoteReplacement(headers));
            FileWriter writer = new FileWriter(curDirectory + "/values.csv", false);
            writer.write(replacedtext);
            writer.close();
        } catch ( Exception e) {
            PLog.fatalException("Logger", "Failed To Update New Headers", e);
        }
    }

    private static String getCurrentTimeFormatted() {
        return dateFormat.format(new Date(System.currentTimeMillis()));
    }

    private static String getSimpleCurrentTimeFormatted() {
        return simpleDateFormat.format(new Date(System.currentTimeMillis()));
    }

    private static String getBaseDirectory() { 
        if (Robot.isSimulation()) 
            return "src/main/Logs/";

        for (String potential : m_DriveRoots) { 
            // Files.exists only works if usb has not been plugged in and removed since first boot, so we have to do this!
            File testDir = new File(potential + "iotest/");
            try {
                testDir.mkdir();
                if (!testDir.exists()) {
                    continue;
                }
                testDir.delete();
                return potential;
            } catch (IOError e) {
                continue;
            }
        }

        return "home/lvuser/Logs/";
    }

    private static String getDirectory() { 
        String mBaseDirectory = getBaseDirectory();
            
        File rootDirectory = new File(mBaseDirectory);
        if (!rootDirectory.isDirectory()) {
            rootDirectory.mkdir();
        }

        // count up previous logging session and number this session accordingly
        Integer maxNum = 0;
        for (final File entry : rootDirectory.listFiles()) {
            try {
                if (!entry.isDirectory()) {
                    continue;
                }
                String directory_name = entry.getName();
                int char_index = directory_name.indexOf(")");
                int num = Integer.parseInt(directory_name.substring(1, char_index));
                if (num > maxNum) {
                    maxNum = num;
                }
            } catch (Exception e) {
                // Files that are not numbers are expected and ignored
            }
        }
        maxNum++;

        char matchType;
        switch (DriverStation.getMatchType()) {
          case Practice:
            matchType = 'P';
            break;
          case Qualification:
            matchType = 'Q';
            break;
          case Elimination:
            matchType = 'E';
            break;
          default:
            matchType = '_';
            break;
        }
        if (DriverStation.isFMSAttached()) {
            return String.format("%s(%d) %s: %s %d", mBaseDirectory, maxNum, DriverStation.getEventName(), matchType, DriverStation.getMatchNumber()); 
        } else {
            return String.format("%s(%d) %s", mBaseDirectory, maxNum, getCurrentTimeFormatted()); 
        }
    }

}
