// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.cad;

public class CadMeshValidationException extends RuntimeException {
  public CadMeshValidationException(String message) { super(message); }
  public CadMeshValidationException(String message, Throwable cause) { super(message, cause); }
}
