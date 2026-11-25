// Copyright (c) 2026 Yet Another Software Suite / YACSim Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yacsim.core.cad;

/**
 * Thrown when a CAD mesh file is missing, unsupported, malformed, or exceeds CPU-parsed triangle
 * budget limits.
 */
public class CadMeshValidationException extends RuntimeException {
  /**
   * Creates a validation exception with a descriptive message.
   *
   * @param message Detail message explaining the validation failure.
   */
  public CadMeshValidationException(String message) { super(message); }
  /**
   * Creates a validation exception with a message and underlying cause.
   *
   * @param message Detail message explaining the validation failure.
   * @param cause Underlying exception.
   */
  public CadMeshValidationException(String message, Throwable cause) { super(message, cause); }
}
