# Makefile pour la compilation de la documentation technique LaTeX FastDrink S5-GI

DOC_NAME = main
BUILD_DIR = build
PDF_NAME = FastDrink_System_Documentation.pdf

all: $(BUILD_DIR)/$(DOC_NAME).pdf
	@cp $(BUILD_DIR)/$(DOC_NAME).pdf ./$(PDF_NAME)
	@echo "=========================================================="
	@echo "Documentation générée avec succès : ./$(PDF_NAME)"
	@echo "=========================================================="

$(BUILD_DIR)/$(DOC_NAME).pdf: $(DOC_NAME).tex chapters/*.tex
	@mkdir -p $(BUILD_DIR)/chapters
	@echo "[1/3] Première passe pdflatex..."
	@pdflatex -interaction=nonstopmode -output-directory=$(BUILD_DIR) $(DOC_NAME).tex > $(BUILD_DIR)/pdflatex.log 2>&1 || (tail -n 40 $(BUILD_DIR)/pdflatex.log && exit 1)
	@echo "[2/3] Deuxième passe pdflatex (mise à jour des références)..."
	@pdflatex -interaction=nonstopmode -output-directory=$(BUILD_DIR) $(DOC_NAME).tex > /dev/null 2>&1
	@echo "[3/3] Troisième passe pdflatex (consolidation de la table des matières)..."
	@pdflatex -interaction=nonstopmode -output-directory=$(BUILD_DIR) $(DOC_NAME).tex > /dev/null 2>&1

clean:
	@rm -rf $(BUILD_DIR)
	@rm -f $(PDF_NAME)
	@echo "Nettoyage des fichiers temporaires terminé."

.PHONY: all clean
