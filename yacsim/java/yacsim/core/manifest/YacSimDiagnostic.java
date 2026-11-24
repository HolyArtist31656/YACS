// Copyright (c) Yet Another Software Suite / YACSim Contributors
// Open Source Software; you can modify and/or share it under the terms of
// the GNU Lesser General Public License v3.0 (LGPL-3.0) in the root directory.

package yacsim.core.manifest;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import yacsim.core.model.CadComponentConfig;
import yacsim.core.model.CadRobotConfig;

/**
 * Startup diagnostic reporter for {@link CadRobotConfig} and {@link CadRobotManifest}, helping
 * teams spot CAD-to-code naming mismatches, unbound joints, and active OBB collision counts in one
 * glance.
 */
public final class YacSimDiagnostic {
  private YacSimDiagnostic() {}

  /**
   * Result of an automatic mechanism binding pass (such as {@code YamsCommands3Bridge.autoBind}),
   * detailing which mechanisms bound cleanly, which Java names had no matching CAD joint, and which
   * CAD joints remain unbound.
   *
   * @param boundNames Java mechanism names that bound cleanly to a CAD joint/component.
   * @param unmatchedJavaNames Java mechanism names that did not match any CAD joint in the manifest.
   * @param unboundCadJoints Non-fixed CAD joints in the manifest that are not yet bound.
   * @param summaryText Human-readable multi-line diagnostic report with alias hints.
   */
  public record BindingReport(
      List<String> boundNames,
      List<String> unmatchedJavaNames,
      List<String> unboundCadJoints,
      String summaryText) {
    /** Compact constructor ensuring immutable non-null lists. */
    public BindingReport {
      boundNames = boundNames == null ? List.of() : List.copyOf(boundNames);
      unmatchedJavaNames =
          unmatchedJavaNames == null ? List.of() : List.copyOf(unmatchedJavaNames);
      unboundCadJoints = unboundCadJoints == null ? List.of() : List.copyOf(unboundCadJoints);
      summaryText = summaryText == null ? "" : summaryText;
    }

    /** @return {@code true} if every supplied Java mechanism name matched a CAD joint. */
    public boolean allMatched() {
      return unmatchedJavaNames.isEmpty();
    }
  }

  /**
   * Suggests the best-matching CAD joint name for an unmatched Java mechanism name using
   * lightweight token-overlap, substring, and prefix similarity across both each joint's name and
   * its child link name (for example matching {@code "Turret"} to {@code "revolute_1"} when its
   * child link is {@code "am_5038_turntable_gear"}).
   *
   * @param javaMechanismName Unmatched Java mechanism name.
   * @param candidates Candidate {@link CadJointManifest} entries to score.
   * @return Best-matching CAD joint name if similarity exceeds threshold, or empty.
   */
  public static Optional<String> suggestBestJointMatch(
      String javaMechanismName, List<CadJointManifest> candidates) {
    if (javaMechanismName == null || javaMechanismName.isBlank() || candidates == null) {
      return Optional.empty();
    }
    String qNorm = CadRobotManifest.normalizeName(javaMechanismName);
    Set<String> qTokens = tokenize(javaMechanismName);
    String bestJoint = null;
    double bestScore = 0.0;

    for (CadJointManifest j : candidates) {
      if (j.type() == CadJointType.FIXED) {
        continue;
      }
      double score =
          Math.max(
              similarityScore(qNorm, qTokens, j.name()),
              similarityScore(qNorm, qTokens, j.childLink()));
      if (score > bestScore) {
        bestScore = score;
        bestJoint = j.name();
      }
    }
    return bestScore >= 0.35 ? Optional.ofNullable(bestJoint) : Optional.empty();
  }

  private static double similarityScore(String qNorm, Set<String> qTokens, String candidate) {
    if (candidate == null || candidate.isBlank()) {
      return 0.0;
    }
    String cNorm = CadRobotManifest.normalizeName(candidate);
    if (cNorm.isEmpty() || qNorm.isEmpty()) {
      return 0.0;
    }
    if (cNorm.equals(qNorm)) {
      return 1.0;
    }
    double score = 0.0;
    if (cNorm.contains(qNorm) || qNorm.contains(cNorm)) {
      score = Math.max(score, 0.85);
    }
    Set<String> cTokens = tokenize(candidate);
    if (!qTokens.isEmpty() && !cTokens.isEmpty()) {
      int overlap = 0;
      for (String qt : qTokens) {
        for (String ct : cTokens) {
          if (qt.equals(ct)) {
            overlap++;
            break;
          }
          if (qt.length() >= 3 && ct.length() >= 3) {
            int prefixLen = commonPrefixLength(qt, ct);
            if (prefixLen >= 3 || ct.contains(qt) || qt.contains(ct)) {
              overlap++;
              break;
            }
          }
        }
      }
      double jaccardLike = (double) overlap / Math.max(1, qTokens.size());
      score = Math.max(score, jaccardLike * 0.8);
    }
    if (qNorm.length() >= 4 && cNorm.length() >= 4) {
      int prefix = commonPrefixLength(qNorm, cNorm);
      if (prefix >= 4) {
        score = Math.max(score, 0.6);
      }
    }
    return score;
  }

  private static Set<String> tokenize(String raw) {
    Set<String> tokens = new LinkedHashSet<>();
    String spaced = raw.replaceAll("([a-z])([A-Z])", "$1 $2");
    for (String part : spaced.toLowerCase(Locale.ROOT).split("[^a-z0-9]+")) {
      if (part.length() >= 2) {
        tokens.add(part);
      }
    }
    return tokens;
  }

  private static int commonPrefixLength(String a, String b) {
    int max = Math.min(a.length(), b.length());
    int i = 0;
    while (i < max && a.charAt(i) == b.charAt(i)) {
      i++;
    }
    return i;
  }

  /**
   * Formats a readable startup diagnostic report for {@code config}, listing chassis, visual
   * components, active OBB collision count, every CAD joint's bound/unbound status, and
   * copy-pasteable {@link CadUrdfOptions#withJointAlias(String, String)} hints for unbound joints.
   *
   * @param config Configured {@link CadRobotConfig}.
   * @return Multi-line diagnostic report.
   */
  public static String formatReport(CadRobotConfig config) {
    Objects.requireNonNull(config, "config");
    StringBuilder sb = new StringBuilder();
    List<CadComponentConfig> comps = config.getAllArticulatedComponents();
    long collisionCount = comps.stream().filter(CadComponentConfig::isCollisionEnabled).count();
    long boundCount =
        comps.stream().filter(c -> config.isComponentBound(c.getName())).count();

    sb.append("=== YACSim Diagnostic Report: ")
        .append(config.getRobotName())
        .append(" ===\n");
    sb.append("  Chassis:    ")
        .append(
            config
                .getChassisAsset()
                .map(a -> a.getMeshPath().getFileName().toString())
                .orElse("(none)"))
        .append('\n');
    sb.append("  Components: ")
        .append(comps.size())
        .append(" visual (")
        .append(boundCount)
        .append(" bound, ")
        .append(collisionCount)
        .append(" collision-enabled)\n");
    sb.append("  Drivetrain: ")
        .append(config.getDrivetrainConfig().isPresent() ? "attached" : "pose-only")
        .append('\n');

    if (config.getManifest().isPresent()) {
      CadRobotManifest manifest = config.getManifest().get();
      List<CadJointManifest> movable =
          manifest.joints().stream().filter(j -> j.type() != CadJointType.FIXED).toList();
      sb.append("  CAD Joints: ").append(movable.size()).append(" movable\n");

      List<String> unboundSuggestions = new ArrayList<>();
      int shown = 0;
      for (CadJointManifest j : movable) {
        boolean isBound = config.isComponentBound(j.childLink());
        if (shown < 25) {
          String status = isBound ? "[BOUND]  " : "[UNBOUND]";
          String limits =
              (j.minLimit().isPresent() || j.maxLimit().isPresent())
                  ? String.format(
                      Locale.ROOT,
                      " limits=[%s, %s]",
                      j.minLimit().isPresent()
                          ? String.format(Locale.ROOT, "%.3f", j.minLimit().getAsDouble())
                          : "-",
                      j.maxLimit().isPresent()
                          ? String.format(Locale.ROOT, "%.3f", j.maxLimit().getAsDouble())
                          : "-")
                  : "";
          String xyz =
              config
                  .getComponentPoseSupplier(j.childLink())
                  .map(
                      s -> {
                        var p = s.get();
                        return String.format(
                            Locale.ROOT,
                            " @ (%.2f, %.2f, %.2f)m",
                            p.getX(),
                            p.getY(),
                            p.getZ());
                      })
                  .orElse("");
          sb.append("    ")
              .append(status)
              .append(' ')
              .append(j.name())
              .append(" (")
              .append(j.type().name().toLowerCase(Locale.ROOT))
              .append(", axis=")
              .append(j.axis())
              .append(") : ")
              .append(j.parentLink())
              .append(" -> ")
              .append(j.childLink())
              .append(xyz)
              .append(limits)
              .append('\n');
          shown++;
        }
        if (!isBound && config.hasComponent(j.childLink()) && unboundSuggestions.size() < 5) {
          unboundSuggestions.add(j.name());
        }
      }
      if (movable.size() > shown) {
        sb.append("    ... (")
            .append(movable.size() - shown)
            .append(" more joints omitted; use CadUrdfOptions.recommended().keepOnlyJoints(...) to trim)\n");
      }
      if (!unboundSuggestions.isEmpty()) {
        sb.append("  Hint: Map default CAD names without editing Onshape:\n");
        sb.append("    CadUrdfOptions.recommended()");
        for (String s : unboundSuggestions) {
          sb.append("\n        .withJointAlias(\"").append(s).append("\", \"<YourMechanism>\")");
        }
        sb.append('\n');
      }
    }
    return sb.toString();
  }

  /**
   * Prints {@link #formatReport(CadRobotConfig)} to standard output.
   *
   * @param config Configured {@link CadRobotConfig}.
   */
  public static void printReport(CadRobotConfig config) {
    System.out.print(formatReport(config));
  }
}
