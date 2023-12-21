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

.venv/bin/python ./indexrepo.py --githubtoken "$GITHUB_TOKEN"\
                                --owner "$GITHUB_OWNER"\
                                --repo "$GITHUB_REPO"\
                                --branch "$GITHUB_BRANCH"\
                                --docs "$GITHUB_DOCS"\
                                --nodes "$GITHUB_NODES"\
                                --azuregptdeployment "$AZURE_OPENAI_GPT_35_DEPLOYMENT"\
                                --azureopenaikey "$AZURE_OPENAI_KEY"\
                                --azureopenaibase "https://$AZURE_OPENAI_SERVICE.openai.azure.com"\
                                --azureopenaiversion "$OPENAI_API_VERSION"\
                                --azureembeddingdeployment "$AZURE_OPENAI_EMB_DEPLOYMENT"\
                                --tenantid "$AZURE_TENANT_ID"\
                                --searchservice "$AZURE_SEARCH_SERVICE"\
                                --searchkey "$AZURE_SEARCH_SERVICE_KEY"\
                                --indexrepo "$AZURE_SEARCH_INDEX_REPO"\
                                -v