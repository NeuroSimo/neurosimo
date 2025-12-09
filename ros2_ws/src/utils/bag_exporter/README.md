## Bag analysis

Start by running:

```
export PROJECT_NAME=...
export BAG_ANALYSIS_BAG_NAME=...
export BAG_ANALYSIS_TIMESTAMP=...
export BAG_ANALYSIS_TOPIC=...
docker-compose -f docker-compose.dev.yml up bag_exporter
```

where the environment variables are set as follows:

# PROJECT_NAME

The project name under projects directory in which the bag is searched for.

# BAG_ANALYSIS_BAG_NAME

For example, set to 'session1' to analyze the bag with that name.

# BAG_ANALYSIS_TIMESTAMP

Optional; together with BAG_ANALYSIS_BAG_NAME, specifies the bag to be analyzed. For example, if
BAG_ANALYSIS_BAG_NAME is session1 and BAG_ANALYSIS_TIMESTAMP is 2023-04-26_11-26-52, analyzes the bag stored in directory:

`projects/[project-name]/bags/2023-04-26_11-26-52-session1`

If not given, use the latest bag matching the bag name.

# BAG_ANALYSIS_TOPIC

The topic to export when analyzing bags. An example value: /eeg/cleaned.
