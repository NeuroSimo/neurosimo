### Exporting bags

- To export ROS bags to CSV, run:

```
docker-compose up bag_exporter
```

- Modify `.env` file in repository root to change the configuration:

* `BAG_ANALYSIS_BAG_NAME`: The bag name for exporting the CSV.
* `BAG_ANALYSIS_TIMESTAMP`: Optional. If not given, use the latest bag with the given name for exporting. An example value: `2023-04-26_11-26-52`.
* `BAG_ANALYSIS_TOPIC`: The topic to export, e.g., `/eeg/cleaned`.

- The changes in `.env` file take place after restarting Docker container.
