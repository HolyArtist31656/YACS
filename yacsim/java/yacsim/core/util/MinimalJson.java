// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.util;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Zero-dependency recursive-descent JSON parser and pretty-printer used for glTF 2.0 chunks and
 * AdvantageScope {@code config.json} generation.
 */
public final class MinimalJson {
  private MinimalJson() {}

  /**
   * Parses a JSON string into Java {@code Map}, {@code List}, {@code String}, {@code Number},
   * {@code Boolean}, or {@code null} values.
   *
   * @param json Raw JSON text.
   * @return Parsed Java value tree.
   */
  public static Object parse(String json) {
    Parser parser = new Parser(json);
    Object value = parser.parseValue();
    parser.skipWhitespace();
    if (!parser.eof()) {
      throw new IllegalArgumentException(
          "Unexpected trailing characters in JSON at index " + parser.pos);
    }
    return value;
  }

  /**
   * Serializes a Java value tree ({@code Map}, {@code List}, primitives, {@code null}) to
   * indented JSON.
   *
   * @param value Root Java value.
   * @return Pretty-printed JSON string.
   */
  public static String toPrettyJson(Object value) {
    StringBuilder sb = new StringBuilder();
    writeValue(sb, value, 0);
    sb.append('\n');
    return sb.toString();
  }

  @SuppressWarnings("unchecked")
  private static void writeValue(StringBuilder sb, Object value, int indent) {
    if (value == null) {
      sb.append("null");
    } else if (value instanceof String s) {
      writeString(sb, s);
    } else if (value instanceof Boolean b) {
      sb.append(b);
    } else if (value instanceof Integer || value instanceof Long) {
      sb.append(value);
    } else if (value instanceof Number n) {
      double d = n.doubleValue();
      if (d == Math.rint(d) && Math.abs(d) < 1e15) {
        sb.append(String.format("%.1f", d));
      } else {
        sb.append(d);
      }
    } else if (value instanceof List<?> list) {
      if (list.isEmpty()) {
        sb.append("[]");
        return;
      }
      sb.append("[\n");
      for (int i = 0; i < list.size(); i++) {
        indent(sb, indent + 2);
        writeValue(sb, list.get(i), indent + 2);
        if (i + 1 < list.size()) sb.append(',');
        sb.append('\n');
      }
      indent(sb, indent);
      sb.append(']');
    } else if (value instanceof Map<?, ?> map) {
      if (map.isEmpty()) {
        sb.append("{}");
        return;
      }
      sb.append("{\n");
      int i = 0;
      for (Map.Entry<String, Object> entry : ((Map<String, Object>) map).entrySet()) {
        indent(sb, indent + 2);
        writeString(sb, entry.getKey());
        sb.append(": ");
        writeValue(sb, entry.getValue(), indent + 2);
        if (++i < map.size()) sb.append(',');
        sb.append('\n');
      }
      indent(sb, indent);
      sb.append('}');
    } else {
      throw new IllegalArgumentException("Unsupported JSON type: " + value.getClass().getName());
    }
  }

  private static void indent(StringBuilder sb, int spaces) {
    for (int i = 0; i < spaces; i++) sb.append(' ');
  }

  private static void writeString(StringBuilder sb, String s) {
    sb.append('"');
    for (int i = 0; i < s.length(); i++) {
      char c = s.charAt(i);
      switch (c) {
        case '"' -> sb.append("\\\"");
        case '\\' -> sb.append("\\\\");
        case '\n' -> sb.append("\\n");
        case '\r' -> sb.append("\\r");
        case '\t' -> sb.append("\\t");
        default -> sb.append(c);
      }
    }
    sb.append('"');
  }

  private static final class Parser {
    private final String src;
    private int pos = 0;

    Parser(String src) { this.src = src; }
    boolean eof() { return pos >= src.length(); }

    void skipWhitespace() {
      while (!eof() && Character.isWhitespace(src.charAt(pos))) pos++;
    }

    Object parseValue() {
      skipWhitespace();
      if (eof()) throw new IllegalArgumentException("Unexpected end of JSON input.");
      return switch (src.charAt(pos)) {
        case '{' -> parseObject();
        case '[' -> parseArray();
        case '"' -> parseString();
        case 't' -> parseLiteral("true", Boolean.TRUE);
        case 'f' -> parseLiteral("false", Boolean.FALSE);
        case 'n' -> parseLiteral("null", null);
        default -> parseNumber();
      };
    }

    private Map<String, Object> parseObject() {
      expect('{');
      Map<String, Object> map = new LinkedHashMap<>();
      skipWhitespace();
      if (!eof() && src.charAt(pos) == '}') { pos++; return map; }
      while (true) {
        skipWhitespace();
        String key = parseString();
        skipWhitespace();
        expect(':');
        map.put(key, parseValue());
        skipWhitespace();
        if (!eof() && src.charAt(pos) == ',') pos++;
        else { expect('}'); return map; }
      }
    }

    private List<Object> parseArray() {
      expect('[');
      List<Object> list = new ArrayList<>();
      skipWhitespace();
      if (!eof() && src.charAt(pos) == ']') { pos++; return list; }
      while (true) {
        list.add(parseValue());
        skipWhitespace();
        if (!eof() && src.charAt(pos) == ',') pos++;
        else { expect(']'); return list; }
      }
    }

    private String parseString() {
      expect('"');
      StringBuilder sb = new StringBuilder();
      while (!eof()) {
        char c = src.charAt(pos++);
        if (c == '"') return sb.toString();
        if (c == '\\') {
          if (eof()) throw new IllegalArgumentException("Unterminated escape sequence.");
          char esc = src.charAt(pos++);
          switch (esc) {
            case '"', '\\', '/' -> sb.append(esc);
            case 'b' -> sb.append('\b');
            case 'f' -> sb.append('\f');
            case 'n' -> sb.append('\n');
            case 'r' -> sb.append('\r');
            case 't' -> sb.append('\t');
            case 'u' -> {
              sb.append((char) Integer.parseInt(src.substring(pos, pos + 4), 16));
              pos += 4;
            }
            default -> throw new IllegalArgumentException("Invalid JSON escape: \\" + esc);
          }
        } else {
          sb.append(c);
        }
      }
      throw new IllegalArgumentException("Unterminated JSON string.");
    }

    private Object parseLiteral(String literal, Object value) {
      if (!src.startsWith(literal, pos)) {
        throw new IllegalArgumentException("Expected '" + literal + "' at index " + pos);
      }
      pos += literal.length();
      return value;
    }

    private Double parseNumber() {
      int start = pos;
      if (!eof() && src.charAt(pos) == '-') pos++;
      while (!eof() && Character.isDigit(src.charAt(pos))) pos++;
      if (!eof() && src.charAt(pos) == '.') {
        pos++;
        while (!eof() && Character.isDigit(src.charAt(pos))) pos++;
      }
      if (!eof() && (src.charAt(pos) == 'e' || src.charAt(pos) == 'E')) {
        pos++;
        if (!eof() && (src.charAt(pos) == '+' || src.charAt(pos) == '-')) pos++;
        while (!eof() && Character.isDigit(src.charAt(pos))) pos++;
      }
      return Double.parseDouble(src.substring(start, pos));
    }

    private void expect(char expected) {
      if (eof() || src.charAt(pos) != expected) {
        throw new IllegalArgumentException("Expected '" + expected + "' at index " + pos);
      }
      pos++;
    }
  }
}
