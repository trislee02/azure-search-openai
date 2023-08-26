 #!/bin/sh

echo ""
echo "Loading azd .env file from current environment"
echo ""

while IFS='=' read -r key value; do
    value=$(echo "$value" | sed 's/^"//' | sed 's/"$//')
    export "$key=$value"
done <<EOF
$(azd env get-values)
EOF

echo 'Creating python virtual environment ".venv"'
python3 -m venv .venv

echo 'Installing dependencies from "requirements.txt" into virtual environment'
.venv/bin/python -m pip install -r requirements.txt

echo 'Running "prepdocs.py"'

# Please provide storage key and search key directly if use existing resources instead of executing "azd provision"
# ./scripts/.venv/bin/python ./prepdocs.py './data/*' --storageaccount "$AZURE_STORAGE_ACCOUNT" --container "$AZURE_STORAGE_CONTAINER" --searchservice "$AZURE_SEARCH_SERVICE" --index "$AZURE_SEARCH_INDEX" --formrecognizerservice "$AZURE_FORMRECOGNIZER_SERVICE" --tenantid "$AZURE_TENANT_ID" --openaiapikey "$OPENAI_API_KEY" -v
.venv/bin/python ./prepdocs.py '../data/*' --storagekey "$AZURE_STORAGE_ACCOUNT_KEY" --searchkey "$AZURE_SEARCH_SERVICE_KEY" --storageaccount "$AZURE_STORAGE_ACCOUNT" --container "$AZURE_STORAGE_CONTAINER" --searchservice "$AZURE_SEARCH_SERVICE" --openaiservice "$AZURE_OPENAI_SERVICE" --openaiembeddingdeployment "$AZURE_OPENAI_EMB_DEPLOYMENT" --openaigptdeployment "$AZURE_OPENAI_GPT_DEPLOYMENT" --index "$AZURE_SEARCH_INDEX" --formrecognizerservice "$AZURE_FORMRECOGNIZER_SERVICE" --tenantid "$AZURE_TENANT_ID" -v
