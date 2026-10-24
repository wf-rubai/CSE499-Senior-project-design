
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from urllib.parse import quote
from statsmodels.nonparametric.smoothers_lowess import lowess
from sklearn.metrics import (
    confusion_matrix,
    accuracy_score,
    precision_score,
    recall_score,
    f1_score,
    classification_report
)

# ==========================================
# 1. GOOGLE SHEETS CONFIGURATION
# ==========================================
SHEET_URL = (
    "https://docs.google.com/spreadsheets/d/"
    "YOUR_COMPLETE_SPREADSHEET_ID/edit"
)
SHEET_NAME = "Sheet1"

DISTANCE_COLUMN = "Distance_m"
ACTUAL_COLUMN = "Actual_Human"

MODEL_COLUMNS = {
    "MobileNet SSD": "MobileNet_SSD_Confidence",
    "YOLOv8": "YOLOv8_Confidence",
    "YOLO11": "YOLO11_Confidence",
}

CONFIDENCE_THRESHOLD = 0.50

# Warm, light color theme
COLORS = {
    "MobileNet SSD": "#C76D4B",
    "YOLOv8": "#D5A044",
    "YOLO11": "#788B69",
}

FIGURE_BG = "#FAF6F0"
AXES_BG = "#FFFDF9"
TEXT_COLOR = "#55483F"
GRID_COLOR = "#E8DDD1"

# ==========================================
# 2. LOAD DATA FROM GOOGLE SHEETS
# ==========================================
spreadsheet_id = SHEET_URL.split("/d/")[1].split("/")[0]

csv_url = (
    f"https://docs.google.com/spreadsheets/d/{spreadsheet_id}/gviz/tq"
    f"?tqx=out:csv&sheet={quote(SHEET_NAME)}"
)

df = pd.read_csv(csv_url)

required_columns = [
    DISTANCE_COLUMN,
    ACTUAL_COLUMN,
    *MODEL_COLUMNS.values()
]

missing = [c for c in required_columns if c not in df.columns]

if missing:
    raise ValueError(
        f"Missing columns: {missing}\n"
        f"Available columns: {df.columns.tolist()}"
    )

df[DISTANCE_COLUMN] = pd.to_numeric(
    df[DISTANCE_COLUMN], errors="coerce"
)

for col in MODEL_COLUMNS.values():
    df[col] = pd.to_numeric(df[col], errors="coerce")

df = df.dropna(subset=[DISTANCE_COLUMN]).copy()

# ==========================================
# 3. CONVERT GROUND-TRUTH LABELS TO 0/1
# ==========================================
# 0 = No Human
# 1 = Human

def convert_actual_label(value):
    if pd.isna(value):
        return np.nan

    if isinstance(value, (bool, np.bool_)):
        return int(value)

    if isinstance(value, (int, float, np.integer, np.floating)):
        if value in (0, 1):
            return int(value)
        return np.nan

    value = str(value).strip().lower()

    positive_labels = {
        "1", "true", "yes", "y", "human",
        "person", "present", "detected"
    }

    negative_labels = {
        "0", "false", "no", "n", "no human",
        "no person", "absent", "not present",
        "not human", "none"
    }

    if value in positive_labels:
        return 1

    if value in negative_labels:
        return 0

    return np.nan


df["_Actual_Binary"] = df[ACTUAL_COLUMN].apply(
    convert_actual_label
)


# ==========================================
# 4. SEPARATE PLOTTING AND EVALUATION DATA
# ==========================================

# Scatter plots: exclude 0 m
plot_data = df[df[DISTANCE_COLUMN] > 0].copy()

# Evaluation: use ALL distances, including 0 m
evaluation_data = df.copy()

print(f"Rows available for plotting: {len(plot_data)}")
print(f"Rows available for evaluation: {len(evaluation_data)}")


# ==========================================
# 5. CREATE 3 MODEL COLUMNS, 3 VERTICAL SECTIONS
# ==========================================
fig = plt.figure(figsize=(19, 17), facecolor=FIGURE_BG)

gs = fig.add_gridspec(
    nrows=3,
    ncols=3,
    height_ratios=[1.15, 0.85, 0.85],
    hspace=0.42,
    wspace=0.30
)

for col_index, (model_name, confidence_column) in enumerate(
    MODEL_COLUMNS.items()
):
    color = COLORS[model_name]

    # --------------------------------------
    # SECTION A: DISTANCE VS CONFIDENCE PLOT
    # --------------------------------------
    ax = fig.add_subplot(gs[0, col_index])
    ax.set_facecolor(AXES_BG)

    # Each model filters its own confidence values independently.
    model_plot = plot_data[
        [DISTANCE_COLUMN, confidence_column]
    ].dropna().copy()

    model_plot = model_plot[
        (model_plot[confidence_column] > 0) &
        (model_plot[confidence_column] <= 1)
    ].sort_values(DISTANCE_COLUMN)

    if not model_plot.empty:
        x = model_plot[DISTANCE_COLUMN].to_numpy()
        y = model_plot[confidence_column].to_numpy()

        ax.scatter(
            x, y,
            color=color,
            alpha=0.48,
            s=30,
            edgecolors="white",
            linewidths=0.5,
            label="Observations",
            zorder=3
        )

        # LOWESS trend line
        if len(model_plot) >= 3 and len(np.unique(x)) >= 3:
            trend = lowess(
                y, x,
                frac=0.65,
                it=1,
                return_sorted=True
            )

            ax.plot(
                trend[:, 0],
                trend[:, 1],
                color=color,
                linewidth=2.8,
                label="LOWESS trend",
                zorder=4
            )

        # Mean ± standard deviation in 2 m-wide bins,
        # centered at 2, 4, 6, 8, ... metres.
        intervals = np.arange(
            np.ceil(x.min() / 2) * 2,
            np.floor(x.max() / 2) * 2 + 0.1,
            2
        )

        bin_x, bin_mean, bin_sd = [], [], []

        for distance in intervals:
            values = y[(x >= distance - 1) & (x < distance + 1)]

            if len(values) == 0:
                continue

            bin_x.append(distance)
            bin_mean.append(np.mean(values))
            bin_sd.append(
                np.std(values, ddof=1) if len(values) > 1 else 0
            )

        if bin_x:
            ax.errorbar(
                bin_x,
                bin_mean,
                yerr=bin_sd,
                fmt="D",
                color=TEXT_COLOR,
                ecolor=color,
                elinewidth=1.4,
                capsize=3,
                markersize=4,
                markerfacecolor=AXES_BG,
                label="Mean ± 1 SD",
                zorder=5
            )

    else:
        ax.text(0.5, 0.5,"No valid nonzero confidence data",ha="center", va="center",transform=ax.transAxes,color=TEXT_COLOR,wrap=True)

    ax.set_title(f"{model_name}\nConfidence vs Distance",fontsize=12,fontweight="bold",color=TEXT_COLOR)
    ax.set_xlabel("Distance (m)", color=TEXT_COLOR)
    ax.set_ylabel("Confidence", color=TEXT_COLOR)
    ax.set_ylim(0, 1.05)
    ax.grid(True, color=GRID_COLOR, alpha=0.8)
    ax.tick_params(colors=TEXT_COLOR)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.legend(fontsize=8, frameon=False, loc="best")

    # --------------------------------------
    # SECTION B: CONFUSION MATRIX AT 0 m
    # --------------------------------------
    cm_ax = fig.add_subplot(gs[1, col_index])
    cm_ax.set_facecolor(AXES_BG)

    eval_model = evaluation_data[
        [ACTUAL_COLUMN, "_Actual_Binary", confidence_column]
    ].dropna(
        subset=["_Actual_Binary", confidence_column]
    ).copy()

    # Confidence 0 is retained here because it may indicate
    # a negative prediction. Do not apply the plotting filter.
    eval_model = eval_model[eval_model[confidence_column].between(0, 1)].copy()

    if not eval_model.empty:
        y_true = eval_model["_Actual_Binary"].astype(int).to_numpy()

        y_pred = ( eval_model[confidence_column].to_numpy() >= CONFIDENCE_THRESHOLD ).astype(int)

        cm = confusion_matrix(y_true, y_pred, labels=[0, 1])

        cm_ax.imshow(cm,interpolation="nearest",cmap="YlOrBr")

        cm_ax.set_title(f"Confusion Matrix\n0 m records (n={len(y_true)})",fontsize=11,fontweight="bold",color=TEXT_COLOR)

        cm_ax.set_xticks([0, 1])
        cm_ax.set_yticks([0, 1])
        cm_ax.set_xticklabels(["No Human", "Human"], rotation=15)
        cm_ax.set_yticklabels(["No Human", "Human"])
        cm_ax.set_xlabel("Predicted label", color=TEXT_COLOR)
        cm_ax.set_ylabel("Actual label", color=TEXT_COLOR)
        cm_ax.tick_params(colors=TEXT_COLOR)

        # Annotate each cell
        for row in range(2):
            for column in range(2):
                cm_ax.text(column, row,str(cm[row, column]),ha="center",va="center",fontsize=13,fontweight="bold",color="#4A3526")

    else:
        cm_ax.axis("off")
        cm_ax.text(0.5, 0.5,"No valid evaluation records\nat 0 m",ha="center", va="center",transform=cm_ax.transAxes,color=TEXT_COLOR)

    # --------------------------------------
    # SECTION C: CLASSIFICATION METRICS
    # --------------------------------------
    metric_ax = fig.add_subplot(gs[2, col_index])
    metric_ax.set_facecolor(AXES_BG)
    metric_ax.axis("off")

    if not eval_model.empty:
        accuracy = accuracy_score(y_true, y_pred)
        precision = precision_score(
            y_true, y_pred, zero_division=0
        )
        recall = recall_score(
            y_true, y_pred, zero_division=0
        )
        f1 = f1_score(
            y_true, y_pred, zero_division=0
        )

        tn, fp, fn, tp = cm.ravel()
        specificity = (
            tn / (tn + fp) if (tn + fp) > 0 else np.nan
        )

        metrics_text = (
            f"Classification Metrics\n"
            f"Threshold: {CONFIDENCE_THRESHOLD:.2f}\n"
            f"Evaluation samples: {len(y_true)}\n\n"
            f"Accuracy:     {accuracy:.3f}\n"
            f"Precision:    {precision:.3f}\n"
            f"Recall:       {recall:.3f}\n"
            f"F1-score:     {f1:.3f}\n"
            f"Specificity:  {specificity:.3f}"
            if not np.isnan(specificity)
            else
            f"Classification Metrics\n"
            f"Threshold: {CONFIDENCE_THRESHOLD:.2f}\n"
            f"Evaluation samples: {len(y_true)}\n\n"
            f"Accuracy:     {accuracy:.3f}\n"
            f"Precision:    {precision:.3f}\n"
            f"Recall:       {recall:.3f}\n"
            f"F1-score:     {f1:.3f}\n"
            f"Specificity:  Not defined"
        )

        metric_ax.text(
            0.08, 0.92,
            metrics_text,
            transform=metric_ax.transAxes,
            ha="left",
            va="top",
            fontsize=11,
            color=TEXT_COLOR,
            family="monospace",
            linespacing=1.5,
            bbox=dict(boxstyle="round,pad=0.7",facecolor=AXES_BG,edgecolor=GRID_COLOR)
        )

    else:
        metric_ax.text(0.5, 0.5,"Metrics unavailable:\nno valid 0 m records",ha="center",va="center",color=TEXT_COLOR,transform=metric_ax.transAxes)

# ==========================================
# 6. TITLE AND SAVE
# ==========================================
fig.suptitle("Human Detection Model Evaluation",fontsize=19,fontweight="bold",color=TEXT_COLOR,y=0.985)

plt.savefig("human_detection_evaluation_horizontal.png", dpi=300, bbox_inches="tight", facecolor=FIGURE_BG)

plt.show()
