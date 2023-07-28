## [Azure Cognitive Search - OpenAI API] Changelog

<a name="Modifying the infrastructure"></a>
# Modifying the infrastructure (2023-07-28)

*Breaking Changes*
* Get rid of Azure OpenAI service
* Need to add an azd environment named OPENAI_API_KEY with the OpenAI API Key as value

*What's next*
* Fix bug calling OpenAI Embedding API in `prepdocs.py`