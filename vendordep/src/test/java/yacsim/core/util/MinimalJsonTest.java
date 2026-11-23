// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class MinimalJsonTest {

  @Test
  @SuppressWarnings("unchecked")
  void roundTripsNestedJsonCleanly() {
    Map<String, Object> root = new LinkedHashMap<>();
    root.put("name", "YACSimBot");
    root.put("disableSimplification", false);
    root.put("position", List.of(0.0, 0.25, -0.5));

    String json = MinimalJson.toPrettyJson(root);
    Map<String, Object> parsed = (Map<String, Object>) MinimalJson.parse(json);

    assertEquals("YACSimBot", parsed.get("name"));
    assertFalse((Boolean) parsed.get("disableSimplification"));
    List<Object> pos = (List<Object>) parsed.get("position");
    assertEquals(3, pos.size());
    assertEquals(-0.5, ((Number) pos.get(2)).doubleValue(), 1e-9);
  }
}
