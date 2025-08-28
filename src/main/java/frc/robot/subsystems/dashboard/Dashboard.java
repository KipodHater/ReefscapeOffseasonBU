package frc.robot.subsystems.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import java.util.ArrayList;

// class Category {
//   private String name;
//   private ArrayList<Object> list;

//   Category(String name) {
//     this.name = name;

//     list = new ArrayList<>();
//   }

//   public String getName() {
//     return name;
//   }

//   public void addValue(Object value) {
//     list.add(value);
//   }

//   public void addValues(Object... values) {
//     for (Object value : values) {
//       list.add(value);
//     }
//   }

//   public ArrayList<Object> getValues() {
//     return list;
//   }
// }

public class Dashboard {
  private NetworkTableInstance ntInstance;
  private NetworkTable defaultTable;
  private static Dashboard instance = null;

  private Dashboard() {
    ntInstance = NetworkTableInstance.getDefault();
    defaultTable = ntInstance.getTable("Dashboard");
  }

  public static Dashboard getInstance() {
    if (instance == null) {
      instance = new Dashboard();
    }
    return instance;
  }

  /**
   * Adds a category to the dashboard with the given name and values. The category is represented as
   * a NetworkTable with the name as the key.
   *
   * @param name The name of the category.
   * @param values The values to be added to the category.
   */
  public void addCategory(String name, Object... values) {
    // Category category = new Category(name);
    NetworkTable categoryTable = defaultTable.getSubTable(name);
    NetworkTable list = categoryTable.getSubTable(name + " List");
    // category.addValues(values);
    categoryTable.getEntry("name").setString(name);
    for (int i = 0; i < values.length; i++) {
      list.getEntry(name + " Value" + " " + i).setValue(values[i]);
    }
    flush();
  }

  /**
   * Gets the value of a specific index in a category.
   *
   * @param name The name of the category.
   * @param index The index of the value to retrieve.
   * @return The value at the specified index in the category.
   */
  public Object getCategoryValue(String name, int index) {
    ArrayList<NetworkTableValue> values = getCategoryValues(name);

    return values.get(index).getValue();
  }

  /**
   * Gets all values in a category.
   *
   * @param name The name of the category.
   * @return An ArrayList of NetworkTableValue objects representing the values in the category.
   */
  public ArrayList<NetworkTableValue> getCategoryValues(String name) {
    NetworkTable categoryTable = defaultTable.getSubTable(name);
    NetworkTable list = categoryTable.getSubTable(name + " List");

    ArrayList<NetworkTableValue> values = new ArrayList<>();
    int i = 0;
    while (true) {
      NetworkTableValue value = list.getEntry(name + " Value" + " " + i).getValue();
      if (value.getType().equals(NetworkTableType.kUnassigned) || value.getValue() == null) {
        break;
      }
      values.add(value);
      i++;
    }
    return values;
  }

  /**
   * Adds a value to a specific category.
   *
   * @param categoryName The name of the category to which the value will be added.
   * @param value The value to be added to the category.
   */
  public void addValueToCategory(String categoryName, Object value) {
    NetworkTable categoryTable = defaultTable.getSubTable(categoryName);
    NetworkTable list = categoryTable.getSubTable(categoryName + " List");
    int len = getCategoryValues(categoryName).size();
    list.getEntry(categoryName + " Value" + " " + len).setValue(value);
    flush();
  }

  /**
   * Sets/Updates a value in a specific category at the given index.
   *
   * @param categoryName The name of the category where the value will be set.
   * @param index The index at which the value will be set.
   * @param value The value to be set in the category.
   */
  public void setValueInCategory(String categoryName, int index, Object value) {
    NetworkTable categoryTable = defaultTable.getSubTable(categoryName);
    NetworkTable list = categoryTable.getSubTable(categoryName + " List");
    list.getEntry(categoryName + " Value" + " " + index).setValue(value);
    flush();
  }

  /**
   * Removes a value from a specific category at the given index.
   *
   * @param categoryName The name of the category from which the value will be removed.
   * @param index The index of the value to be removed.
   */
  public void removeValueFromCategory(String categoryName, int index) {
    NetworkTable categoryTable = defaultTable.getSubTable(categoryName);
    NetworkTable list = categoryTable.getSubTable(categoryName + " List");
    list.getEntry(categoryName + " Value" + " " + index).close();
    flush();
  }

  private void flush() {
    ntInstance.flush();
  }
}
